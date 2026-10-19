-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- ribbon.vhd
--
-- RIBBON -- full-screen candy-ribbon stripes.  A standalone fork of the
-- "rainbow taffy" texture from SUGARCOAT, blown up to fill the frame and
-- grown into a full instrument.  No video in -- a pure generator.
--
--   P12 WARP     bend depth, squared, up to a fold-over climax  CONTINUOUS
--   K1  ANGLE    stripe rotation, 0..180 deg   (128 steps, glided)
--   K2  FLOW     bipolar: stripes march + ripples travel; centre = still
--   K3  SCALE    ribbon size, 7 octaves, EXPONENTIAL           CONTINUOUS
--   K4  PALETTE  crossfade through 7 ramps     CONTINUOUS (wraps)
--   K5  RIPPLE   warp wavelength               CONTINUOUS
--   K6  GLOSS    rounding + specular + emboss
--   S7  CHEVRON  mirror the field about screen centre -> arrowheads
--   S8  PLAID    over/under basket weave with a second ribbon set at 90 deg
--   S9  WAVE     Smooth (sine) / Zigzag (triangle) ripple shape
--   S10 ROPE     mirror the palette ramp -> symmetric twisted ropes
--   S11 OUTLINE  dark seam between stops -> individually wrapped candies
--
-- EVERYTHING RIDES ACCUMULATORS.  The stripe projection is not computed from
-- (dx,dy) at all: A advances by `ca` per pixel and by `sa` per active line,
-- so a continuous rotation costs ZERO pixel-path multiplies (see
-- [[videosky]]'s accumulator-pair trick).  The perpendicular weave B and both
-- ripple phases are three more accumulators.  A is in 1/256 px, which is what
-- makes the warp and the zoom smooth rather than stepped.  FLOW is two more
-- per-frame offsets added to the accumulators' frame seeds -- motion with no
-- pixel-path cost at all.
--
-- SCALE is exponential: the octave is the barrel-shift pitch, the mantissa a
-- 256-entry 2^-x zoom table, so every 1/7 of the knob is exactly one octave.
-- WARP is held in PIXELS (its accumulator-domain depth is scaled by the zoom
-- each frame), so Scale never changes the bend and the octave seam is
-- invisible.
--
-- Resolution scaling (v1.0.2): the field is drawn on a virtual 1080-line
-- canvas -- per-pixel steps x xstep (1080 1.0, 720p 1.5, SD 2.0, SD being a
-- 1440-unit 4:3 window), per-line steps once per virtual line crossed (Q4.8
-- y step per field line from the timing ID, x2 for interlace).  Stripe
-- pitch, warp depth and ripple wavelength are the same fraction of screen
-- height at SD, 720p and 1080p; 1080p is bit-identical to v1.0.1.  This
-- replaces the field_n-toggle interlace detect, which never fires on
-- hardware (core_top ties field_n high), so 480i drew 2:1 stretched stripes.
--
-- Hardware contracts: own generated avid (parity-locked start, fixed
-- per-field length -- the U/V-swap line fix from CUBIST), outputs gated to
-- neutral outside it, frame work keyed to "avid absent for 2 line widths"
-- (immune to serrated analog vsync), luma kept legal (palettes 64..780,
-- output clamped 64..940), and K1/K3/K5 glided + deadbanded so pot noise
-- can't shimmer the far corner of the field.
--
-- Pipeline (streaming, C_LATENCY = 16, one line RAM):
--   A      accumulator snapshot + ripple shape (fold + sine ROM)
--   W1..W3 signed bend sum -> one multiply -> added to both projections
--   Q      one barrel shift yields stop index AND sub-stop position
--   ST     rope fold + weave order + outline decision
--   R1/R2  palette register file x2 -> weave select -> outline darken
--   S1/S2  ribbon rounding + specular line from the sub-stop position
--   L0..L4 line RAM -> gx/gy -> lit -> shift -> spec add + clamp
--   O      colour + shade + sheen, clamped legal, U/V swapped for hardware
--   F1/F2  horizontal luma 1-2-1 (no full-swing 1-px steps), re-gated
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture ribbon of program_top is
    -- data_in -> data_out latency is 16 clocks, already a multiple of 4 as the
    -- SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;

    -- pixel-data depth: A(1) W1..W3(4) Q(5) ST(6) R1/R2(8) cy_sr 0..4(13) O(14)
    -- then the luma 1-2-1 (centre tap 15, output register 16)
    constant C_LATENCY : integer := 16;
    -- avid tap aligned with the colour entering cy_sr(4) / g_add: the stage
    -- one before O, where the first blanking gate sits
    constant C_GATE_IDX : integer := 11;

    ----------------------------------------------------------------------
    -- generated tables (genpal.py): legal-level palettes, quarter sine,
    -- one-octave exponential zoom mantissa
    ----------------------------------------------------------------------
    type t_pal56 is array (0 to 55) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal56 := (
        to_unsigned(64, 10), to_unsigned(201, 10), to_unsigned(329, 10), to_unsigned(588, 10), to_unsigned(800, 10), to_unsigned(652, 10), to_unsigned(275, 10), to_unsigned(940, 10),  -- 0: Penny Candy
        to_unsigned(144, 10), to_unsigned(324, 10), to_unsigned(598, 10), to_unsigned(751, 10), to_unsigned(782, 10), to_unsigned(732, 10), to_unsigned(792, 10), to_unsigned(940, 10),  -- 8: Bubblegum
        to_unsigned(104, 10), to_unsigned(242, 10), to_unsigned(381, 10), to_unsigned(305, 10), to_unsigned(494, 10), to_unsigned(774, 10), to_unsigned(940, 10), to_unsigned(940, 10),  -- 16: Peppermint
        to_unsigned(88, 10), to_unsigned(182, 10), to_unsigned(329, 10), to_unsigned(488, 10), to_unsigned(638, 10), to_unsigned(778, 10), to_unsigned(901, 10), to_unsigned(940, 10),  -- 24: Bakery
        to_unsigned(123, 10), to_unsigned(624, 10), to_unsigned(840, 10), to_unsigned(880, 10), to_unsigned(444, 10), to_unsigned(643, 10), to_unsigned(871, 10), to_unsigned(940, 10),  -- 32: Sour
        to_unsigned(109, 10), to_unsigned(606, 10), to_unsigned(734, 10), to_unsigned(868, 10), to_unsigned(606, 10), to_unsigned(700, 10), to_unsigned(692, 10), to_unsigned(940, 10),  -- 40: Tropical Taffy
        to_unsigned(245, 10), to_unsigned(554, 10), to_unsigned(678, 10), to_unsigned(829, 10), to_unsigned(875, 10), to_unsigned(786, 10), to_unsigned(854, 10), to_unsigned(940, 10)  -- 48: Holo Wrapper
    );
    constant C_PAL_U : t_pal56 := (
        to_unsigned(505, 10), to_unsigned(444, 10), to_unsigned(417, 10), to_unsigned(181, 10), to_unsigned(62, 10), to_unsigned(213, 10), to_unsigned(718, 10), to_unsigned(512, 10),  -- 0: Penny Candy
        to_unsigned(555, 10), to_unsigned(578, 10), to_unsigned(571, 10), to_unsigned(541, 10), to_unsigned(546, 10), to_unsigned(676, 10), to_unsigned(642, 10), to_unsigned(509, 10),  -- 8: Bubblegum
        to_unsigned(499, 10), to_unsigned(466, 10), to_unsigned(433, 10), to_unsigned(431, 10), to_unsigned(437, 10), to_unsigned(212, 10), to_unsigned(502, 10), to_unsigned(512, 10),  -- 16: Peppermint
        to_unsigned(481, 10), to_unsigned(450, 10), to_unsigned(404, 10), to_unsigned(362, 10), to_unsigned(345, 10), to_unsigned(345, 10), to_unsigned(389, 10), to_unsigned(461, 10),  -- 24: Bakery
        to_unsigned(465, 10), to_unsigned(206, 10), to_unsigned(39, 10), to_unsigned(62, 10), to_unsigned(713, 10), to_unsigned(647, 10), to_unsigned(293, 10), to_unsigned(512, 10),  -- 32: Sour
        to_unsigned(519, 10), to_unsigned(374, 10), to_unsigned(235, 10), to_unsigned(182, 10), to_unsigned(600, 10), to_unsigned(603, 10), to_unsigned(518, 10), to_unsigned(499, 10),  -- 40: Tropical Taffy
        to_unsigned(532, 10), to_unsigned(652, 10), to_unsigned(672, 10), to_unsigned(519, 10), to_unsigned(359, 10), to_unsigned(521, 10), to_unsigned(562, 10), to_unsigned(512, 10)  -- 48: Holo Wrapper
    );
    constant C_PAL_V : t_pal56 := (
        to_unsigned(589, 10), to_unsigned(769, 10), to_unsigned(905, 10), to_unsigned(821, 10), to_unsigned(670, 10), to_unsigned(391, 10), to_unsigned(573, 10), to_unsigned(512, 10),  -- 0: Penny Candy
        to_unsigned(581, 10), to_unsigned(709, 10), to_unsigned(814, 10), to_unsigned(706, 10), to_unsigned(327, 10), to_unsigned(334, 10), to_unsigned(548, 10), to_unsigned(522, 10),  -- 8: Bubblegum
        to_unsigned(495, 10), to_unsigned(368, 10), to_unsigned(298, 10), to_unsigned(865, 10), to_unsigned(845, 10), to_unsigned(618, 10), to_unsigned(542, 10), to_unsigned(512, 10),  -- 16: Peppermint
        to_unsigned(549, 10), to_unsigned(583, 10), to_unsigned(620, 10), to_unsigned(650, 10), to_unsigned(643, 10), to_unsigned(615, 10), to_unsigned(570, 10), to_unsigned(533, 10),  -- 24: Bakery
        to_unsigned(482, 10), to_unsigned(411, 10), to_unsigned(485, 10), to_unsigned(614, 10), to_unsigned(923, 10), to_unsigned(112, 10), to_unsigned(406, 10), to_unsigned(512, 10),  -- 32: Sour
        to_unsigned(549, 10), to_unsigned(809, 10), to_unsigned(718, 10), to_unsigned(622, 10), to_unsigned(195, 10), to_unsigned(185, 10), to_unsigned(747, 10), to_unsigned(524, 10),  -- 40: Tropical Taffy
        to_unsigned(509, 10), to_unsigned(546, 10), to_unsigned(372, 10), to_unsigned(322, 10), to_unsigned(575, 10), to_unsigned(652, 10), to_unsigned(504, 10), to_unsigned(512, 10)  -- 48: Holo Wrapper
    );
    type t_wave512 is array (0 to 511) of unsigned(7 downto 0);
    constant C_WAVE : t_wave512 := (
        to_unsigned(0, 8), to_unsigned(2, 8), to_unsigned(3, 8), to_unsigned(5, 8), to_unsigned(6, 8), to_unsigned(8, 8), to_unsigned(9, 8), to_unsigned(11, 8), to_unsigned(13, 8), to_unsigned(14, 8), to_unsigned(16, 8), to_unsigned(17, 8), to_unsigned(19, 8), to_unsigned(20, 8), to_unsigned(22, 8), to_unsigned(24, 8),
        to_unsigned(25, 8), to_unsigned(27, 8), to_unsigned(28, 8), to_unsigned(30, 8), to_unsigned(31, 8), to_unsigned(33, 8), to_unsigned(34, 8), to_unsigned(36, 8), to_unsigned(38, 8), to_unsigned(39, 8), to_unsigned(41, 8), to_unsigned(42, 8), to_unsigned(44, 8), to_unsigned(45, 8), to_unsigned(47, 8), to_unsigned(48, 8),
        to_unsigned(50, 8), to_unsigned(51, 8), to_unsigned(53, 8), to_unsigned(55, 8), to_unsigned(56, 8), to_unsigned(58, 8), to_unsigned(59, 8), to_unsigned(61, 8), to_unsigned(62, 8), to_unsigned(64, 8), to_unsigned(65, 8), to_unsigned(67, 8), to_unsigned(68, 8), to_unsigned(70, 8), to_unsigned(71, 8), to_unsigned(73, 8),
        to_unsigned(74, 8), to_unsigned(76, 8), to_unsigned(77, 8), to_unsigned(79, 8), to_unsigned(80, 8), to_unsigned(82, 8), to_unsigned(83, 8), to_unsigned(85, 8), to_unsigned(86, 8), to_unsigned(88, 8), to_unsigned(89, 8), to_unsigned(91, 8), to_unsigned(92, 8), to_unsigned(94, 8), to_unsigned(95, 8), to_unsigned(96, 8),
        to_unsigned(98, 8), to_unsigned(99, 8), to_unsigned(101, 8), to_unsigned(102, 8), to_unsigned(104, 8), to_unsigned(105, 8), to_unsigned(107, 8), to_unsigned(108, 8), to_unsigned(109, 8), to_unsigned(111, 8), to_unsigned(112, 8), to_unsigned(114, 8), to_unsigned(115, 8), to_unsigned(116, 8), to_unsigned(118, 8), to_unsigned(119, 8),
        to_unsigned(121, 8), to_unsigned(122, 8), to_unsigned(123, 8), to_unsigned(125, 8), to_unsigned(126, 8), to_unsigned(127, 8), to_unsigned(129, 8), to_unsigned(130, 8), to_unsigned(132, 8), to_unsigned(133, 8), to_unsigned(134, 8), to_unsigned(136, 8), to_unsigned(137, 8), to_unsigned(138, 8), to_unsigned(140, 8), to_unsigned(141, 8),
        to_unsigned(142, 8), to_unsigned(143, 8), to_unsigned(145, 8), to_unsigned(146, 8), to_unsigned(147, 8), to_unsigned(149, 8), to_unsigned(150, 8), to_unsigned(151, 8), to_unsigned(152, 8), to_unsigned(154, 8), to_unsigned(155, 8), to_unsigned(156, 8), to_unsigned(157, 8), to_unsigned(159, 8), to_unsigned(160, 8), to_unsigned(161, 8),
        to_unsigned(162, 8), to_unsigned(164, 8), to_unsigned(165, 8), to_unsigned(166, 8), to_unsigned(167, 8), to_unsigned(168, 8), to_unsigned(169, 8), to_unsigned(171, 8), to_unsigned(172, 8), to_unsigned(173, 8), to_unsigned(174, 8), to_unsigned(175, 8), to_unsigned(176, 8), to_unsigned(178, 8), to_unsigned(179, 8), to_unsigned(180, 8),
        to_unsigned(181, 8), to_unsigned(182, 8), to_unsigned(183, 8), to_unsigned(184, 8), to_unsigned(185, 8), to_unsigned(186, 8), to_unsigned(187, 8), to_unsigned(188, 8), to_unsigned(190, 8), to_unsigned(191, 8), to_unsigned(192, 8), to_unsigned(193, 8), to_unsigned(194, 8), to_unsigned(195, 8), to_unsigned(196, 8), to_unsigned(197, 8),
        to_unsigned(198, 8), to_unsigned(199, 8), to_unsigned(200, 8), to_unsigned(201, 8), to_unsigned(202, 8), to_unsigned(203, 8), to_unsigned(203, 8), to_unsigned(204, 8), to_unsigned(205, 8), to_unsigned(206, 8), to_unsigned(207, 8), to_unsigned(208, 8), to_unsigned(209, 8), to_unsigned(210, 8), to_unsigned(211, 8), to_unsigned(212, 8),
        to_unsigned(213, 8), to_unsigned(213, 8), to_unsigned(214, 8), to_unsigned(215, 8), to_unsigned(216, 8), to_unsigned(217, 8), to_unsigned(218, 8), to_unsigned(218, 8), to_unsigned(219, 8), to_unsigned(220, 8), to_unsigned(221, 8), to_unsigned(222, 8), to_unsigned(222, 8), to_unsigned(223, 8), to_unsigned(224, 8), to_unsigned(225, 8),
        to_unsigned(225, 8), to_unsigned(226, 8), to_unsigned(227, 8), to_unsigned(228, 8), to_unsigned(228, 8), to_unsigned(229, 8), to_unsigned(230, 8), to_unsigned(230, 8), to_unsigned(231, 8), to_unsigned(232, 8), to_unsigned(232, 8), to_unsigned(233, 8), to_unsigned(234, 8), to_unsigned(234, 8), to_unsigned(235, 8), to_unsigned(235, 8),
        to_unsigned(236, 8), to_unsigned(237, 8), to_unsigned(237, 8), to_unsigned(238, 8), to_unsigned(238, 8), to_unsigned(239, 8), to_unsigned(239, 8), to_unsigned(240, 8), to_unsigned(241, 8), to_unsigned(241, 8), to_unsigned(242, 8), to_unsigned(242, 8), to_unsigned(243, 8), to_unsigned(243, 8), to_unsigned(243, 8), to_unsigned(244, 8),
        to_unsigned(244, 8), to_unsigned(245, 8), to_unsigned(245, 8), to_unsigned(246, 8), to_unsigned(246, 8), to_unsigned(247, 8), to_unsigned(247, 8), to_unsigned(247, 8), to_unsigned(248, 8), to_unsigned(248, 8), to_unsigned(248, 8), to_unsigned(249, 8), to_unsigned(249, 8), to_unsigned(249, 8), to_unsigned(250, 8), to_unsigned(250, 8),
        to_unsigned(250, 8), to_unsigned(251, 8), to_unsigned(251, 8), to_unsigned(251, 8), to_unsigned(251, 8), to_unsigned(252, 8), to_unsigned(252, 8), to_unsigned(252, 8), to_unsigned(252, 8), to_unsigned(253, 8), to_unsigned(253, 8), to_unsigned(253, 8), to_unsigned(253, 8), to_unsigned(253, 8), to_unsigned(254, 8), to_unsigned(254, 8),
        to_unsigned(254, 8), to_unsigned(254, 8), to_unsigned(254, 8), to_unsigned(254, 8), to_unsigned(254, 8), to_unsigned(255, 8), to_unsigned(255, 8), to_unsigned(255, 8), to_unsigned(255, 8), to_unsigned(255, 8), to_unsigned(255, 8), to_unsigned(255, 8), to_unsigned(255, 8), to_unsigned(255, 8), to_unsigned(255, 8), to_unsigned(255, 8),
        to_unsigned(0, 8), to_unsigned(1, 8), to_unsigned(2, 8), to_unsigned(3, 8), to_unsigned(4, 8), to_unsigned(5, 8), to_unsigned(6, 8), to_unsigned(7, 8), to_unsigned(8, 8), to_unsigned(9, 8), to_unsigned(10, 8), to_unsigned(11, 8), to_unsigned(12, 8), to_unsigned(13, 8), to_unsigned(14, 8), to_unsigned(15, 8),
        to_unsigned(16, 8), to_unsigned(17, 8), to_unsigned(18, 8), to_unsigned(19, 8), to_unsigned(20, 8), to_unsigned(21, 8), to_unsigned(22, 8), to_unsigned(23, 8), to_unsigned(24, 8), to_unsigned(25, 8), to_unsigned(26, 8), to_unsigned(27, 8), to_unsigned(28, 8), to_unsigned(29, 8), to_unsigned(30, 8), to_unsigned(31, 8),
        to_unsigned(32, 8), to_unsigned(33, 8), to_unsigned(34, 8), to_unsigned(35, 8), to_unsigned(36, 8), to_unsigned(37, 8), to_unsigned(38, 8), to_unsigned(39, 8), to_unsigned(40, 8), to_unsigned(41, 8), to_unsigned(42, 8), to_unsigned(43, 8), to_unsigned(44, 8), to_unsigned(45, 8), to_unsigned(46, 8), to_unsigned(47, 8),
        to_unsigned(48, 8), to_unsigned(49, 8), to_unsigned(50, 8), to_unsigned(51, 8), to_unsigned(52, 8), to_unsigned(53, 8), to_unsigned(54, 8), to_unsigned(55, 8), to_unsigned(56, 8), to_unsigned(57, 8), to_unsigned(58, 8), to_unsigned(59, 8), to_unsigned(60, 8), to_unsigned(61, 8), to_unsigned(62, 8), to_unsigned(63, 8),
        to_unsigned(64, 8), to_unsigned(65, 8), to_unsigned(66, 8), to_unsigned(67, 8), to_unsigned(68, 8), to_unsigned(69, 8), to_unsigned(70, 8), to_unsigned(71, 8), to_unsigned(72, 8), to_unsigned(73, 8), to_unsigned(74, 8), to_unsigned(75, 8), to_unsigned(76, 8), to_unsigned(77, 8), to_unsigned(78, 8), to_unsigned(79, 8),
        to_unsigned(80, 8), to_unsigned(81, 8), to_unsigned(82, 8), to_unsigned(83, 8), to_unsigned(84, 8), to_unsigned(85, 8), to_unsigned(86, 8), to_unsigned(87, 8), to_unsigned(88, 8), to_unsigned(89, 8), to_unsigned(90, 8), to_unsigned(91, 8), to_unsigned(92, 8), to_unsigned(93, 8), to_unsigned(94, 8), to_unsigned(95, 8),
        to_unsigned(96, 8), to_unsigned(97, 8), to_unsigned(98, 8), to_unsigned(99, 8), to_unsigned(100, 8), to_unsigned(101, 8), to_unsigned(102, 8), to_unsigned(103, 8), to_unsigned(104, 8), to_unsigned(105, 8), to_unsigned(106, 8), to_unsigned(107, 8), to_unsigned(108, 8), to_unsigned(109, 8), to_unsigned(110, 8), to_unsigned(111, 8),
        to_unsigned(112, 8), to_unsigned(113, 8), to_unsigned(114, 8), to_unsigned(115, 8), to_unsigned(116, 8), to_unsigned(117, 8), to_unsigned(118, 8), to_unsigned(119, 8), to_unsigned(120, 8), to_unsigned(121, 8), to_unsigned(122, 8), to_unsigned(123, 8), to_unsigned(124, 8), to_unsigned(125, 8), to_unsigned(126, 8), to_unsigned(127, 8),
        to_unsigned(128, 8), to_unsigned(129, 8), to_unsigned(130, 8), to_unsigned(131, 8), to_unsigned(132, 8), to_unsigned(133, 8), to_unsigned(134, 8), to_unsigned(135, 8), to_unsigned(136, 8), to_unsigned(137, 8), to_unsigned(138, 8), to_unsigned(139, 8), to_unsigned(140, 8), to_unsigned(141, 8), to_unsigned(142, 8), to_unsigned(143, 8),
        to_unsigned(144, 8), to_unsigned(145, 8), to_unsigned(146, 8), to_unsigned(147, 8), to_unsigned(148, 8), to_unsigned(149, 8), to_unsigned(150, 8), to_unsigned(151, 8), to_unsigned(152, 8), to_unsigned(153, 8), to_unsigned(154, 8), to_unsigned(155, 8), to_unsigned(156, 8), to_unsigned(157, 8), to_unsigned(158, 8), to_unsigned(159, 8),
        to_unsigned(160, 8), to_unsigned(161, 8), to_unsigned(162, 8), to_unsigned(163, 8), to_unsigned(164, 8), to_unsigned(165, 8), to_unsigned(166, 8), to_unsigned(167, 8), to_unsigned(168, 8), to_unsigned(169, 8), to_unsigned(170, 8), to_unsigned(171, 8), to_unsigned(172, 8), to_unsigned(173, 8), to_unsigned(174, 8), to_unsigned(175, 8),
        to_unsigned(176, 8), to_unsigned(177, 8), to_unsigned(178, 8), to_unsigned(179, 8), to_unsigned(180, 8), to_unsigned(181, 8), to_unsigned(182, 8), to_unsigned(183, 8), to_unsigned(184, 8), to_unsigned(185, 8), to_unsigned(186, 8), to_unsigned(187, 8), to_unsigned(188, 8), to_unsigned(189, 8), to_unsigned(190, 8), to_unsigned(191, 8),
        to_unsigned(192, 8), to_unsigned(193, 8), to_unsigned(194, 8), to_unsigned(195, 8), to_unsigned(196, 8), to_unsigned(197, 8), to_unsigned(198, 8), to_unsigned(199, 8), to_unsigned(200, 8), to_unsigned(201, 8), to_unsigned(202, 8), to_unsigned(203, 8), to_unsigned(204, 8), to_unsigned(205, 8), to_unsigned(206, 8), to_unsigned(207, 8),
        to_unsigned(208, 8), to_unsigned(209, 8), to_unsigned(210, 8), to_unsigned(211, 8), to_unsigned(212, 8), to_unsigned(213, 8), to_unsigned(214, 8), to_unsigned(215, 8), to_unsigned(216, 8), to_unsigned(217, 8), to_unsigned(218, 8), to_unsigned(219, 8), to_unsigned(220, 8), to_unsigned(221, 8), to_unsigned(222, 8), to_unsigned(223, 8),
        to_unsigned(224, 8), to_unsigned(225, 8), to_unsigned(226, 8), to_unsigned(227, 8), to_unsigned(228, 8), to_unsigned(229, 8), to_unsigned(230, 8), to_unsigned(231, 8), to_unsigned(232, 8), to_unsigned(233, 8), to_unsigned(234, 8), to_unsigned(235, 8), to_unsigned(236, 8), to_unsigned(237, 8), to_unsigned(238, 8), to_unsigned(239, 8),
        to_unsigned(240, 8), to_unsigned(241, 8), to_unsigned(242, 8), to_unsigned(243, 8), to_unsigned(244, 8), to_unsigned(245, 8), to_unsigned(246, 8), to_unsigned(247, 8), to_unsigned(248, 8), to_unsigned(249, 8), to_unsigned(250, 8), to_unsigned(251, 8), to_unsigned(252, 8), to_unsigned(253, 8), to_unsigned(254, 8), to_unsigned(255, 8)
    );
    type t_zoom256 is array (0 to 255) of unsigned(9 downto 0);
    constant C_ZOOM : t_zoom256 := (
        to_unsigned(512, 10), to_unsigned(511, 10), to_unsigned(509, 10), to_unsigned(508, 10), to_unsigned(506, 10), to_unsigned(505, 10), to_unsigned(504, 10), to_unsigned(502, 10), to_unsigned(501, 10), to_unsigned(500, 10), to_unsigned(498, 10), to_unsigned(497, 10), to_unsigned(496, 10), to_unsigned(494, 10), to_unsigned(493, 10), to_unsigned(492, 10),
        to_unsigned(490, 10), to_unsigned(489, 10), to_unsigned(488, 10), to_unsigned(486, 10), to_unsigned(485, 10), to_unsigned(484, 10), to_unsigned(482, 10), to_unsigned(481, 10), to_unsigned(480, 10), to_unsigned(478, 10), to_unsigned(477, 10), to_unsigned(476, 10), to_unsigned(475, 10), to_unsigned(473, 10), to_unsigned(472, 10), to_unsigned(471, 10),
        to_unsigned(470, 10), to_unsigned(468, 10), to_unsigned(467, 10), to_unsigned(466, 10), to_unsigned(464, 10), to_unsigned(463, 10), to_unsigned(462, 10), to_unsigned(461, 10), to_unsigned(459, 10), to_unsigned(458, 10), to_unsigned(457, 10), to_unsigned(456, 10), to_unsigned(454, 10), to_unsigned(453, 10), to_unsigned(452, 10), to_unsigned(451, 10),
        to_unsigned(450, 10), to_unsigned(448, 10), to_unsigned(447, 10), to_unsigned(446, 10), to_unsigned(445, 10), to_unsigned(444, 10), to_unsigned(442, 10), to_unsigned(441, 10), to_unsigned(440, 10), to_unsigned(439, 10), to_unsigned(438, 10), to_unsigned(436, 10), to_unsigned(435, 10), to_unsigned(434, 10), to_unsigned(433, 10), to_unsigned(432, 10),
        to_unsigned(431, 10), to_unsigned(429, 10), to_unsigned(428, 10), to_unsigned(427, 10), to_unsigned(426, 10), to_unsigned(425, 10), to_unsigned(424, 10), to_unsigned(422, 10), to_unsigned(421, 10), to_unsigned(420, 10), to_unsigned(419, 10), to_unsigned(418, 10), to_unsigned(417, 10), to_unsigned(416, 10), to_unsigned(415, 10), to_unsigned(413, 10),
        to_unsigned(412, 10), to_unsigned(411, 10), to_unsigned(410, 10), to_unsigned(409, 10), to_unsigned(408, 10), to_unsigned(407, 10), to_unsigned(406, 10), to_unsigned(405, 10), to_unsigned(403, 10), to_unsigned(402, 10), to_unsigned(401, 10), to_unsigned(400, 10), to_unsigned(399, 10), to_unsigned(398, 10), to_unsigned(397, 10), to_unsigned(396, 10),
        to_unsigned(395, 10), to_unsigned(394, 10), to_unsigned(393, 10), to_unsigned(392, 10), to_unsigned(391, 10), to_unsigned(389, 10), to_unsigned(388, 10), to_unsigned(387, 10), to_unsigned(386, 10), to_unsigned(385, 10), to_unsigned(384, 10), to_unsigned(383, 10), to_unsigned(382, 10), to_unsigned(381, 10), to_unsigned(380, 10), to_unsigned(379, 10),
        to_unsigned(378, 10), to_unsigned(377, 10), to_unsigned(376, 10), to_unsigned(375, 10), to_unsigned(374, 10), to_unsigned(373, 10), to_unsigned(372, 10), to_unsigned(371, 10), to_unsigned(370, 10), to_unsigned(369, 10), to_unsigned(368, 10), to_unsigned(367, 10), to_unsigned(366, 10), to_unsigned(365, 10), to_unsigned(364, 10), to_unsigned(363, 10),
        to_unsigned(362, 10), to_unsigned(361, 10), to_unsigned(360, 10), to_unsigned(359, 10), to_unsigned(358, 10), to_unsigned(357, 10), to_unsigned(356, 10), to_unsigned(355, 10), to_unsigned(354, 10), to_unsigned(353, 10), to_unsigned(352, 10), to_unsigned(351, 10), to_unsigned(350, 10), to_unsigned(350, 10), to_unsigned(349, 10), to_unsigned(348, 10),
        to_unsigned(347, 10), to_unsigned(346, 10), to_unsigned(345, 10), to_unsigned(344, 10), to_unsigned(343, 10), to_unsigned(342, 10), to_unsigned(341, 10), to_unsigned(340, 10), to_unsigned(339, 10), to_unsigned(338, 10), to_unsigned(337, 10), to_unsigned(337, 10), to_unsigned(336, 10), to_unsigned(335, 10), to_unsigned(334, 10), to_unsigned(333, 10),
        to_unsigned(332, 10), to_unsigned(331, 10), to_unsigned(330, 10), to_unsigned(329, 10), to_unsigned(328, 10), to_unsigned(328, 10), to_unsigned(327, 10), to_unsigned(326, 10), to_unsigned(325, 10), to_unsigned(324, 10), to_unsigned(323, 10), to_unsigned(322, 10), to_unsigned(321, 10), to_unsigned(321, 10), to_unsigned(320, 10), to_unsigned(319, 10),
        to_unsigned(318, 10), to_unsigned(317, 10), to_unsigned(316, 10), to_unsigned(315, 10), to_unsigned(314, 10), to_unsigned(314, 10), to_unsigned(313, 10), to_unsigned(312, 10), to_unsigned(311, 10), to_unsigned(310, 10), to_unsigned(309, 10), to_unsigned(309, 10), to_unsigned(308, 10), to_unsigned(307, 10), to_unsigned(306, 10), to_unsigned(305, 10),
        to_unsigned(304, 10), to_unsigned(304, 10), to_unsigned(303, 10), to_unsigned(302, 10), to_unsigned(301, 10), to_unsigned(300, 10), to_unsigned(300, 10), to_unsigned(299, 10), to_unsigned(298, 10), to_unsigned(297, 10), to_unsigned(296, 10), to_unsigned(296, 10), to_unsigned(295, 10), to_unsigned(294, 10), to_unsigned(293, 10), to_unsigned(292, 10),
        to_unsigned(292, 10), to_unsigned(291, 10), to_unsigned(290, 10), to_unsigned(289, 10), to_unsigned(288, 10), to_unsigned(288, 10), to_unsigned(287, 10), to_unsigned(286, 10), to_unsigned(285, 10), to_unsigned(285, 10), to_unsigned(284, 10), to_unsigned(283, 10), to_unsigned(282, 10), to_unsigned(281, 10), to_unsigned(281, 10), to_unsigned(280, 10),
        to_unsigned(279, 10), to_unsigned(278, 10), to_unsigned(278, 10), to_unsigned(277, 10), to_unsigned(276, 10), to_unsigned(275, 10), to_unsigned(275, 10), to_unsigned(274, 10), to_unsigned(273, 10), to_unsigned(272, 10), to_unsigned(272, 10), to_unsigned(271, 10), to_unsigned(270, 10), to_unsigned(270, 10), to_unsigned(269, 10), to_unsigned(268, 10),
        to_unsigned(267, 10), to_unsigned(267, 10), to_unsigned(266, 10), to_unsigned(265, 10), to_unsigned(264, 10), to_unsigned(264, 10), to_unsigned(263, 10), to_unsigned(262, 10), to_unsigned(262, 10), to_unsigned(261, 10), to_unsigned(260, 10), to_unsigned(259, 10), to_unsigned(259, 10), to_unsigned(258, 10), to_unsigned(257, 10), to_unsigned(257, 10)
    );

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
    function f_cu10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return resize(unsigned(v), 10);
        end if;
    end function;

    -- output clamp: legal luma only.  Full-scale Y clips all RGB and kills
    -- chroma on hardware; below 64 is below black.
    function f_cleg(v : signed) return unsigned is
    begin
        if v < 64 then
            return to_unsigned(64, 10);
        elsif v > 940 then
            return to_unsigned(940, 10);
        else
            return resize(unsigned(v), 10);
        end if;
    end function;

    -- Ripple-shape table INDEX: zig & folded quarter position (8 bits from a
    -- 10-bit phase, 1024 = one cycle; the sign is a(9)).  Zigzag = the folded
    -- triangle (exactly linear inside each quarter); Smooth = a quarter sine,
    -- so the ribbons undulate instead of kinking.  256 levels per quarter:
    -- with 64 (v0.3.0) each level was a 3-6 px jump at full Warp and the
    -- contours stair-stepped into flickering torn lines.
    function f_widx(a : unsigned(9 downto 0); zig : std_logic) return unsigned is
        variable v_i : unsigned(7 downto 0);
    begin
        if a(8) = '0' then v_i := a(7 downto 0);
        else               v_i := not a(7 downto 0); end if;
        return zig & v_i;
    end function;

    ----------------------------------------------------------------------
    -- Resolution scaling (v1.0.2): the field is drawn on a virtual 1080-line
    -- canvas.  Per-pixel steps x xstep (1080 1.0, 720p 1.5, SD 2.0 -- SD is a
    -- 1440-unit 4:3 window), per-line steps advance once per virtual line
    -- crossed (Q4.8 y step per FIELD line from the timing ID, x2 for
    -- interlace).  At 1080p both are 1 and the picture is unchanged.
    ----------------------------------------------------------------------
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);  -- 480i: 2 x 1080/486
            when 4          => return to_unsigned(576, 12);   -- 480p: 1080/480
            when 8          => return to_unsigned(960, 12);   -- 576i: 2 x 1080/576
            when 12         => return to_unsigned(480, 12);   -- 576p: 1080/576
            when 5 | 6 | 14 => return to_unsigned(384, 12);   -- 720p: 1.5
            when 1 | 2 | 10 => return to_unsigned(512, 12);   -- 1080i: 2.0
            when others     => return to_unsigned(256, 12);   -- 1080p: 1.0
        end case;
    end function;

    -- signed step x xstep: + v (SD), + v/2 (720p), + 0 (1080)
    function f_xss(v : signed; sd, h7 : std_logic) return signed is
        variable r : signed(v'length downto 0);
    begin
        r := resize(v, v'length + 1);
        if sd = '1' then
            return r + r;
        elsif h7 = '1' then
            return r + shift_right(r, 1);
        else
            return r;
        end if;
    end function;

    ----------------------------------------------------------------------
    -- quarter cosine table: C_COS(k) = round(362 * cos(k * 90/64 deg)).
    -- 362 = 256*sqrt(2), so at 45 deg ca = sa = 256 exactly -- i.e. the
    -- accumulator matches a plain (dx+dy) diagonal at unity zoom.
    ----------------------------------------------------------------------
    type t_cos65 is array (0 to 64) of signed(10 downto 0);
    constant C_COS : t_cos65 := (
        to_signed(362, 11), to_signed(362, 11), to_signed(362, 11), to_signed(361, 11), to_signed(360, 11), to_signed(359, 11), to_signed(358, 11), to_signed(357, 11),   -- 0
        to_signed(355, 11), to_signed(353, 11), to_signed(351, 11), to_signed(349, 11), to_signed(346, 11), to_signed(344, 11), to_signed(341, 11), to_signed(338, 11),   -- 8
        to_signed(334, 11), to_signed(331, 11), to_signed(327, 11), to_signed(323, 11), to_signed(319, 11), to_signed(315, 11), to_signed(310, 11), to_signed(306, 11),   -- 16
        to_signed(301, 11), to_signed(296, 11), to_signed(291, 11), to_signed(285, 11), to_signed(280, 11), to_signed(274, 11), to_signed(268, 11), to_signed(262, 11),   -- 24
        to_signed(256, 11), to_signed(250, 11), to_signed(243, 11), to_signed(236, 11), to_signed(230, 11), to_signed(223, 11), to_signed(216, 11), to_signed(208, 11),   -- 32
        to_signed(201, 11), to_signed(194, 11), to_signed(186, 11), to_signed(178, 11), to_signed(171, 11), to_signed(163, 11), to_signed(155, 11), to_signed(147, 11),   -- 40
        to_signed(139, 11), to_signed(130, 11), to_signed(122, 11), to_signed(114, 11), to_signed(105, 11), to_signed(97, 11), to_signed(88, 11), to_signed(79, 11),   -- 48
        to_signed(71, 11), to_signed(62, 11), to_signed(53, 11), to_signed(44, 11), to_signed(35, 11), to_signed(27, 11), to_signed(18, 11), to_signed(9, 11),   -- 56
        to_signed(0, 11)   -- 64
    );

    -- Full-circle cosine from the quarter table.  i is a 256-step angle
    -- (1.40625 deg/step).  Folded to ONE table read (four separate reads
    -- would instance the ROM four times).
    function f_cos(i : unsigned(7 downto 0)) return signed is
        variable k : integer range 0 to 64;
        variable v : signed(10 downto 0);
    begin
        if i(6) = '0' then k := to_integer(i(5 downto 0));
        else               k := 64 - to_integer(i(5 downto 0)); end if;
        v := C_COS(k);
        if (i(7) xor i(6)) = '1' then return -v; else return v; end if;
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_k2 : unsigned(9 downto 0) := to_unsigned(600, 10);   -- Flow
    signal s_k4 : unsigned(9 downto 0) := (others => '0');        -- Palette
    signal s_k6 : unsigned(9 downto 0) := to_unsigned(640, 10);   -- Gloss

    signal s_chev  : std_logic := '0';   -- S7
    signal s_plaid : std_logic := '0';   -- S8
    signal s_zig   : std_logic := '0';   -- S9
    signal s_rope  : std_logic := '0';   -- S10
    signal s_outl  : std_logic := '0';   -- S11

    -- GLIDE + DEADBAND.  The field is anchored at the top-left, so one LSB of
    -- pot noise on Scale or Ripple moves the far corner by ~8 px.  K1/K3/K5
    -- are held in 10.4 fixed point and eased by diff>>3 per frame, and only
    -- when the knob is more than 2.5 LSB away -- noise never moves them, a
    -- deliberate turn glides with sub-LSB smoothness.  P12 is deadbanded only
    -- (the performance slider stays immediate).
    signal s_t1, s_t3, s_t5 : unsigned(13 downto 0) := (others => '0');
    signal s_z1 : unsigned(13 downto 0) := to_unsigned(256 * 16, 14);
    signal s_z3 : unsigned(13 downto 0) := to_unsigned(293 * 16, 14);
    signal s_z5 : unsigned(13 downto 0) := to_unsigned(250 * 16, 14);
    signal s_d1, s_d3, s_d5 : signed(14 downto 0) := (others => '0');
    signal s_g1, s_g3, s_g5 : signed(14 downto 0) := (others => '0');
    signal s_tp : unsigned(9 downto 0) := to_unsigned(320, 10);
    signal s_zp : unsigned(9 downto 0) := to_unsigned(320, 10);
    signal s_dp : signed(10 downto 0) := (others => '0');

    -- raw quarter-table reads, then zoom-scaled step sizes
    signal s_c0, s_s0 : signed(10 downto 0) := (others => '0');
    signal s_L    : unsigned(16 downto 0) := (others => '0');          -- K3 * 7
    signal s_zq   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_oct  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_zoom : unsigned(9 downto 0) := to_unsigned(512, 10);      -- 257..512
    signal s_ca : signed(10 downto 0) := to_signed(256, 11);           -- x step
    signal s_sa : signed(10 downto 0) := to_signed(256, 11);           -- y step
    -- barrel shift that takes the 1/256-px accumulator to (stop, sub-stop):
    -- 7 + SCALE octave
    signal s_qsh : integer range 7 to 13 := 9;
    signal s_wsq   : unsigned(19 downto 0) := (others => '0');
    -- ONE shared per-frame multiplier (cos*zoom, sin*zoom, warp^2,
    -- warp*zoom): free-running, product valid 2 cycles after the operands
    -- are set.  Four separate ones cost ~300 LC of quasi-static logic.
    signal m_a : signed(11 downto 0) := (others => '0');
    signal m_b : signed(10 downto 0) := (others => '0');
    signal m_p : signed(22 downto 0) := (others => '0');
    signal s_wamp  : unsigned(11 downto 0) := to_unsigned(400, 12);    -- WARP depth
    signal s_wstep : unsigned(14 downto 0) := to_unsigned(4128, 15);   -- RIPPLE step
    signal s_gsh   : integer range 0 to 5 := 5;                        -- emboss shift
    signal s_gspec : unsigned(8 downto 0) := to_unsigned(220, 9);      -- emboss spec thr
    signal s_gdif  : unsigned(7 downto 0) := to_unsigned(160, 8);      -- ribbon rounding
    signal s_gspc  : unsigned(9 downto 0) := to_unsigned(320, 10);     -- specular line

    -- FLOW: |K2 - 512| -> piecewise-exponential speed -> per-frame steps
    signal s_fneg  : std_logic := '0';
    signal s_fabs  : unsigned(8 downto 0) := (others => '0');
    signal s_fmag  : unsigned(11 downto 0) := (others => '0');
    signal s_astep : unsigned(16 downto 0) := (others => '0');
    signal s_wvs   : unsigned(13 downto 0) := (others => '0');
    signal s_aoff  : signed(22 downto 0) := (others => '0');    -- stripe march
    signal s_woff  : unsigned(19 downto 0) := (others => '0');  -- ripple travel
    -- Resolution scaling: timing class, y step, the per-line virtual-line
    -- repeat counter, and the x-scaled per-pixel steps.  (Replaces v1.0.1's
    -- field_n-toggle interlace detect: core_top ties field_n high, so it
    -- never fired on hardware and 480i stripes were stretched 2:1.)
    signal s_sd, s_h7 : std_logic := '0';
    signal s_ys    : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_yf    : unsigned(7 downto 0) := (others => '0');
    signal s_yrep  : unsigned(2 downto 0) := (others => '0');
    signal s_cax   : signed(11 downto 0) := to_signed(256, 12);
    signal s_sax   : signed(11 downto 0) := to_signed(256, 12);
    signal s_wsx   : unsigned(15 downto 0) := to_unsigned(4128, 16);

    -- blended palette register file (K4 crossfade result), read by the pixel
    -- path as a plain 8:1 mux -- no ROM, no multiply, no ramp maths per pixel.
    type t_pal8 is array (0 to 7) of unsigned(9 downto 0);
    -- init chroma to neutral: an all-zero chroma RAM renders a saturated
    -- green wall, not black, if anything ever samples it pre-fill.
    signal s_ply : t_pal8 := (others => to_unsigned(64, 10));
    signal s_plu : t_pal8 := (others => to_unsigned(512, 10));
    signal s_plv : t_pal8 := (others => to_unsigned(512, 10));
    signal s_offa, s_offb : unsigned(5 downto 0) := (others => '0');
    signal s_pmix : unsigned(5 downto 0) := (others => '0');   -- crossfade t, 0..63
    -- crossfade engine working registers
    signal b_ay, b_au, b_av : unsigned(9 downto 0) := (others => '0');
    signal b_by, b_bu, b_bv : unsigned(9 downto 0) := (others => '0');
    signal b_dy, b_du, b_dv : signed(10 downto 0) := (others => '0');
    signal b_py, b_pu, b_pv : signed(17 downto 0) := (others => '0');
    signal s_bi  : unsigned(2 downto 0) := (others => '0');
    signal s_bph : unsigned(2 downto 0) := (others => '0');
    signal s_bdone : std_logic := '0';
    -- final blended value + PRE-DECODED one-hot write enable.  The hop into
    -- the 240-flop palette file must be flop -> flop with no logic: that file
    -- places scattered, and driving it through an add+clamp AND a decoded
    -- enable was 10.6 ns of pure ROUTING on the critical path.
    signal b_wy, b_wu, b_wv : unsigned(9 downto 0) := (others => '0');
    signal b_we : std_logic_vector(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- raster measurement + generated avid + frame start
    ----------------------------------------------------------------------
    signal s_avid_q     : std_logic := '0';
    signal s_avid_r     : std_logic := '0';
    signal s_hs_q, s_hs_r : std_logic := '0';
    signal s_xcnt       : unsigned(11 downto 0) := (others => '0');
    signal s_W          : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_Wl         : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_Wlm1       : unsigned(11 downto 0) := to_unsigned(719, 12);
    signal s_cxu        : unsigned(11 downto 0) := to_unsigned(360, 12);
    signal s_idle       : unsigned(12 downto 0) := (others => '0');
    signal s_earm       : std_logic := '0';
    signal s_fstart     : std_logic := '0';
    -- generated avid: parity-locked start, ends after the field-latched width
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avid_q : std_logic := '0';
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc  : unsigned(3 downto 0) := "1000";
    signal g_x   : unsigned(11 downto 0) := (others => '0');

    signal s_seq : unsigned(4 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- projection + ripple accumulators (no multiplies anywhere here)
    ----------------------------------------------------------------------
    signal s_newframe : std_logic := '1';
    signal s_row0     : std_logic := '0';   -- first active row of the frame
    signal s_arow, s_apx : signed(22 downto 0) := (others => '0');  -- ribbons
    signal s_brow, s_bpx : signed(22 downto 0) := (others => '0');  -- cross weave
    signal s_wphx, s_wphy : unsigned(19 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- pattern pipeline
    ----------------------------------------------------------------------
    signal a_ap, a_bp : signed(22 downto 0) := (others => '0');
    signal a_mx, a_my : unsigned(7 downto 0) := (others => '0');
    signal a_sx, a_sy : std_logic := '0';
    signal a_x        : unsigned(10 downto 0) := (others => '0');
    signal w1_ts      : signed(9 downto 0) := (others => '0');
    signal w1_ap, w1_bp : signed(22 downto 0) := (others => '0');
    signal w1_x       : unsigned(10 downto 0) := (others => '0');
    signal w2_ow      : signed(19 downto 0) := (others => '0');
    signal w2_ap, w2_bp : signed(22 downto 0) := (others => '0');
    signal w2_x       : unsigned(10 downto 0) := (others => '0');
    signal w3_ap, w3_bp : signed(22 downto 0) := (others => '0');
    signal w3_x       : unsigned(10 downto 0) := (others => '0');
    signal q_a, q_b   : unsigned(8 downto 0) := (others => '0');
    signal q_x        : unsigned(10 downto 0) := (others => '0');
    signal st_sa, st_sb   : unsigned(2 downto 0) := (others => '0');
    signal st_ua, st_ub   : unsigned(4 downto 0) := (others => '0');
    signal st_ol, st_top  : std_logic := '0';
    signal st_x       : unsigned(10 downto 0) := (others => '0');
    signal ra_y, ra_u, ra_v : unsigned(9 downto 0) := (others => '0');
    signal rb_y, rb_u, rb_v : unsigned(9 downto 0) := (others => '0');
    signal r1_ol, r1_top : std_logic := '0';
    signal r1_ua, r1_ub : unsigned(4 downto 0) := (others => '0');
    signal r1_x       : unsigned(10 downto 0) := (others => '0');
    signal r_y, r_u, r_v : unsigned(9 downto 0) := (others => '0');
    signal r_sub      : unsigned(4 downto 0) := (others => '0');
    signal r_x        : unsigned(10 downto 0) := (others => '0');

    -- ribbon rounding / specular (from the sub-stop position)
    signal sh_d, sh_s : signed(10 downto 0) := (others => '0');
    signal sh        : signed(11 downto 0) := (others => '0');

    -- ribbon colour delayed to meet the sheen; the shade folds in at index 2
    type t_csr is array (0 to 4) of unsigned(9 downto 0);
    signal cy_sr : t_csr := (others => to_unsigned(64, 10));
    signal cu_sr, cv_sr : t_csr := (others => to_unsigned(512, 10));

    ----------------------------------------------------------------------
    -- GLOSS emboss: the ribbons' own (flat) luma is the heightfield, so
    -- every fold edge picks up a lit / shadowed seam.  The ROUNDING above
    -- is what gives the ribbon its body; this puts the crease on it.
    ----------------------------------------------------------------------
    type t_lram is array (0 to 2047) of std_logic_vector(9 downto 0);
    signal lram   : t_lram;
    signal lb_rd  : std_logic_vector(9 downto 0) := (others => '0');
    signal lw_en  : std_logic := '0';
    signal lw_x   : unsigned(10 downto 0) := (others => '0');
    signal lw_y   : unsigned(9 downto 0) := (others => '0');
    signal gpl, gpl_d : unsigned(9 downto 0) := (others => '0');
    signal ggx, ggy : signed(10 downto 0) := (others => '0');
    signal glit     : signed(11 downto 0) := (others => '0');
    signal spec_hi  : std_logic := '0';
    signal spec_hi2 : std_logic := '0';
    signal g_sh     : signed(13 downto 0) := (others => '0');
    signal g_add    : signed(10 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- output
    ----------------------------------------------------------------------
    signal o_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- luma 1-2-1 taps + final (re-gated) outputs
    signal f_y1, f_y2, f_out_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal f_u1, f_v1, f_out_u, f_out_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal o_u, o_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- raster measurement, GENERATED AVID and frame start.
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;   -- falling edge of _n

            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then
                if s_xcnt > 16 then s_W <= s_xcnt; end if;
                s_xcnt <= (others => '0');
            end if;

            -- CLEAN AVID (CUBIST's HW-confirmed fix).  The encoders get no
            -- data-enable -- only hsync and the 4:2:2 bus -- so they pair
            -- Cb/Cr by counting from hsync, while the core's 4:4:4->4:2:2
            -- packer restarts on Cb at every avid rise.  A source avid that
            -- wanders a clock against hsync (or drops out mid-line) swaps U/V
            -- for that line: thin coloured lines over coloured areas -- and
            -- RIBBON is coloured wall to wall.  So the program makes its own
            -- avid: it starts 2 or 3 clocks after the source's avid rise,
            -- whichever keeps the hsync->start distance at the source's USUAL
            -- parity (a saturating vote, so jitter can't move it), and ends
            -- after the field-latched width, so dropouts are ignored.
            if s_hs_r = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if s_avid_r = '1' and g_arm = '1' then
                g_arm    <= '0';
                g_rsr(0) <= '1';
                g_sel    <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;
            g_avid_q <= g_avid;
            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
            elsif g_avid = '1' and g_x = s_Wlm1 then
                g_avid <= '0';
            end if;
            if g_avid = '1' then
                g_x <= g_x + 1;
            else
                g_x <= (others => '0');
            end if;

            -- FRAME START: active video absent for two line widths -- early
            -- in vertical blanking.  Not vsync: an analog vsync serrates
            -- (several edges per field), which re-ran the interlace detect
            -- and flipped it off.  Re-arms only on active video.
            s_fstart <= '0';
            if data_in.avid = '1' then
                s_idle <= (others => '0');
                s_earm <= '1';
            else
                if s_idle /= "1111111111111" then s_idle <= s_idle + 1; end if;
                if s_earm = '1' and s_idle = (s_W & '0') then
                    s_fstart <= '1';
                    s_earm   <= '0';
                end if;
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer, then the K4 palette
    -- CROSSFADE engine which fills the 8-entry palette register file the
    -- pixel path reads.  Both run only while avid = '0'.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_idx : unsigned(7 downto 0);
        variable v_p7  : unsigned(12 downto 0);
        variable v_q   : unsigned(8 downto 0);
        variable v_pa  : integer range 0 to 6;
        variable v_pb  : integer range 0 to 6;
        variable v_i   : integer range 0 to 7;
        variable v_c   : integer;
        variable v_zs  : signed(14 downto 0);
        variable v_wx  : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            m_p <= m_a * m_b;
            if s_fstart = '1' then
                s_t1  <= unsigned(registers_in(0)) & "0000";
                s_k2  <= unsigned(registers_in(1));
                s_t3  <= unsigned(registers_in(2)) & "0000";
                s_k4  <= unsigned(registers_in(3));
                s_t5  <= unsigned(registers_in(4)) & "0000";
                s_k6  <= unsigned(registers_in(5));
                s_tp  <= unsigned(registers_in(7));
                s_chev   <= registers_in(6)(0);        -- S7
                s_plaid  <= registers_in(6)(1);        -- S8
                s_zig    <= registers_in(6)(2);        -- S9
                s_rope   <= registers_in(6)(3);        -- S10
                s_outl   <= registers_in(6)(4);        -- S11
                s_Wl  <= s_W;
                s_bi    <= (others => '0');
                s_bph   <= (others => '0');
                s_bdone <= '0';
                b_we    <= (others => '0');
                s_seq   <= to_unsigned(24, 5);
            elsif data_in.avid = '0' then
                if s_seq /= 0 then
                    -- ONE derived term per cycle: per-frame logic is timed at
                    -- the pixel clock, so a fat cone here caps Fmax exactly
                    -- like a pixel-path one.
                    case to_integer(s_seq) is
                        when 24 =>
                            -- GLIDE 1/3: distance to the knob
                            s_d1 <= signed('0' & s_t1) - signed('0' & s_z1);
                            s_d3 <= signed('0' & s_t3) - signed('0' & s_z3);
                            s_d5 <= signed('0' & s_t5) - signed('0' & s_z5);
                            s_dp <= signed('0' & s_tp) - signed('0' & s_zp);
                        when 23 =>
                            -- GLIDE 2/3: ease step, zero inside the deadband
                            if s_d1 > 40 or s_d1 < -40 then s_g1 <= shift_right(s_d1, 3);
                            else                            s_g1 <= (others => '0'); end if;
                            if s_d3 > 40 or s_d3 < -40 then s_g3 <= shift_right(s_d3, 3);
                            else                            s_g3 <= (others => '0'); end if;
                            if s_d5 > 40 or s_d5 < -40 then s_g5 <= shift_right(s_d5, 3);
                            else                            s_g5 <= (others => '0'); end if;
                            if s_dp > 1 or s_dp < -1 then s_zp <= s_tp; end if;
                        when 22 =>
                            -- GLIDE 3/3
                            v_zs := signed('0' & s_z1) + s_g1;
                            s_z1 <= unsigned(v_zs(13 downto 0));
                            v_zs := signed('0' & s_z3) + s_g3;
                            s_z3 <= unsigned(v_zs(13 downto 0));
                            v_zs := signed('0' & s_z5) + s_g5;
                            s_z5 <= unsigned(v_zs(13 downto 0));
                        when 21 =>
                            -- ANGLE (K1): 128 steps over 0..180 deg.  Both
                            -- quarter-table reads happen here; the zoom
                            -- multiply is a SEPARATE cycle.
                            v_idx := '0' & s_z1(13 downto 7);
                            s_c0 <= f_cos(v_idx);
                            s_s0 <= f_cos(v_idx + 192);      -- sin = cos(-90)
                            s_Wlm1 <= s_Wl - 1;
                            s_cxu  <= '0' & s_Wl(11 downto 1);
                        when 20 =>
                            -- SCALE (K3): 7 octaves over the knob.  L = K3*7
                            -- in 10.4 -> octave = L(16:14), mantissa L(13:6).
                            s_L <= shift_left(resize(s_z3, 17), 3) - resize(s_z3, 17);
                        when 19 =>
                            s_zq  <= C_ZOOM(to_integer(s_L(13 downto 6)));
                            s_oct <= s_L(16 downto 14);
                        when 18 =>
                            -- second register on the table read (a ROM may map
                            -- to EBR and absorb the first one)
                            s_zoom <= s_zq;
                            s_qsh  <= 7 + to_integer(s_oct);
                        when 17 =>
                            m_a <= resize(s_c0, 12);
                            m_b <= signed(resize(s_zoom, 11));
                        when 16 =>
                            m_a <= resize(s_s0, 12);
                        when 15 =>
                            s_ca <= resize(shift_right(m_p, 8), 11);
                            -- WARP (P12): SQUARED, so the low end stays fine
                            -- and the top folds the ribbons over themselves.
                            m_a <= signed(resize(s_zp, 12));
                            m_b <= signed(resize(s_zp, 11));
                        when 14 =>
                            s_sa <= resize(shift_right(m_p, 8), 11);
                        when 13 =>
                            s_wsq <= unsigned(m_p(19 downto 0));
                        when 12 =>
                            -- ...held in PIXELS: the accumulator moves zoom
                            -- units per pixel, so scale the depth by the zoom
                            -- (Scale then never changes the bend, and the
                            -- octave seam can't jump it).
                            m_a <= signed(resize(s_wsq(19 downto 9), 12));
                            m_b <= signed(resize(s_zoom, 11));
                        when 11 =>
                            null;
                        when 10 =>
                            s_wamp <= unsigned(m_p(19 downto 8));     -- slice: <= 4094 overflows a signed resize
                        when 9 =>
                            -- RIPPLE (K5): warp wavelength.  step 128..16511
                            -- of a 2^20 phase = ~8000 px down to ~64 px.
                            s_wstep <= resize(s_z5, 15) + 128;
                            v_c := to_integer(s_k6(9 downto 7));
                            if v_c > 5 then v_c := 5; end if;
                            s_gsh   <= v_c;
                            s_gspec <= to_unsigned(300, 9) - resize(s_k6(9 downto 3), 9);
                        when 8 =>
                            -- GLOSS body: how hard each ribbon is rounded and
                            -- how hot the specular line running along it is.
                            s_gdif <= s_k6(9 downto 2);
                            s_gspc <= '0' & s_k6(9 downto 1);
                        when 7 =>
                            -- FLOW (K2): bipolar about 512, no adder needed
                            s_fneg <= not s_k2(9);
                            if s_k2(9) = '1' then s_fabs <= s_k2(8 downto 0);
                            else                  s_fabs <= not s_k2(8 downto 0); end if;
                        when 6 =>
                            -- piecewise-exponential speed, deadband at centre
                            if s_fabs < 16 then
                                s_fmag <= (others => '0');
                            else
                                -- resize BEFORE the +16: a 4-bit operand wraps it
                                s_fmag <= shift_left(resize(s_fabs(5 downto 2), 12) + 16,
                                                     to_integer(s_fabs(8 downto 6)));
                            end if;
                        when 5 =>
                            -- stripes march in STOPS per frame at any Scale:
                            -- a stop is 2^(12+oct) accumulator units
                            s_astep <= shift_left(resize(s_fmag(11 downto 2), 17),
                                                  to_integer(s_oct));
                            s_wvs   <= s_fmag & "00";
                        when 4 =>
                            if s_fneg = '1' then
                                s_aoff <= s_aoff - signed(resize(s_astep, 23));
                                s_woff <= s_woff - resize(s_wvs, 20);
                            else
                                s_aoff <= s_aoff + signed(resize(s_astep, 23));
                                s_woff <= s_woff + resize(s_wvs, 20);
                            end if;
                        when 3 =>
                            -- resolution scaling: per-pixel steps x xstep
                            s_cax <= f_xss(s_ca, s_sd, s_h7);
                            s_sax <= f_xss(s_sa, s_sd, s_h7);
                            v_wx  := f_xss(signed('0' & s_wstep), s_sd, s_h7);
                            s_wsx <= unsigned(v_wx(15 downto 0));   -- <= 33022: slice
                        when 2 =>
                            -- PALETTE (K4): position through the 7 ramps, and
                            -- the crossfade weight to the NEXT one (wraps 6->0
                            -- so the knob is a continuous loop).
                            -- NB: unsigned*natural returns DOUBLE width, so
                            -- resize explicitly rather than assuming 13 bits.
                            v_p7 := resize(s_k4 * 7, 13);              -- 0..7161
                            v_q  := resize(shift_right(v_p7, 4), 9);   -- 0..447
                            v_pa := to_integer(v_q(8 downto 6));
                            if v_pa >= 6 then v_pb := 0; else v_pb := v_pa + 1; end if;
                            s_offa <= to_unsigned(v_pa * 8, 6);
                            s_offb <= to_unsigned(v_pb * 8, 6);
                            s_pmix <= v_q(5 downto 0);
                        when others =>
                            null;
                    end case;
                    s_seq <= s_seq - 1;
                elsif s_bdone = '0' then
                    -- PALETTE CROSSFADE, 5 cycles per stop.  Every step is
                    -- deliberately thin: fetch, SUBTRACT (its own cycle -- with
                    -- it feeding the multiplier's partial products this was the
                    -- design's critical path at 13.6 ns), MULTIPLY, form the
                    -- result + one-hot enable, then a bare flop-to-flop write.
                    v_i := to_integer(s_bi);
                    case to_integer(s_bph) is
                        when 0 =>
                            b_ay <= C_PAL_Y(to_integer(s_offa) + v_i);
                            b_au <= C_PAL_U(to_integer(s_offa) + v_i);
                            b_av <= C_PAL_V(to_integer(s_offa) + v_i);
                            b_by <= C_PAL_Y(to_integer(s_offb) + v_i);
                            b_bu <= C_PAL_U(to_integer(s_offb) + v_i);
                            b_bv <= C_PAL_V(to_integer(s_offb) + v_i);
                        when 1 =>
                            b_dy <= signed(resize(b_by, 11)) - signed(resize(b_ay, 11));
                            b_du <= signed(resize(b_bu, 11)) - signed(resize(b_au, 11));
                            b_dv <= signed(resize(b_bv, 11)) - signed(resize(b_av, 11));
                        when 2 =>
                            b_py <= b_dy * signed(resize(s_pmix, 7));
                            b_pu <= b_du * signed(resize(s_pmix, 7));
                            b_pv <= b_dv * signed(resize(s_pmix, 7));
                        when 3 =>
                            b_wy <= f_cu10(signed(resize(b_ay, 13))
                                           + resize(shift_right(b_py, 6), 13));
                            b_wu <= f_cu10(signed(resize(b_au, 13))
                                           + resize(shift_right(b_pu, 6), 13));
                            b_wv <= f_cu10(signed(resize(b_av, 13))
                                           + resize(shift_right(b_pv, 6), 13));
                            b_we <= (others => '0');
                            b_we(v_i) <= '1';
                        when others =>
                            -- the only cycle that touches the palette file
                            for k in 0 to 7 loop
                                if b_we(k) = '1' then
                                    s_ply(k) <= b_wy;
                                    s_plu(k) <= b_wu;
                                    s_plv(k) <= b_wv;
                                end if;
                            end loop;
                            b_we <= (others => '0');
                    end case;
                    if s_bph = 4 then
                        s_bph <= (others => '0');
                        if s_bi = 7 then s_bdone <= '1';
                        else             s_bi <= s_bi + 1; end if;
                    else
                        s_bph <= s_bph + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- ACCUMULATORS.  A = the ribbon projection (rotates with K1), B = the
    -- perpendicular weave, wphx/wphy = the two ripple phases.  All four are
    -- pure adders: no multiply is needed for rotation, zoom or ripple.
    -- Seeded each frame at the FLOW offsets, advanced on ACTIVE lines only
    -- (interlace-safe -- see the DDA/field phase trap), driven by the
    -- generated avid, and mirrored past screen centre when CHEVRON is on.
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_ar, v_br : signed(22 downto 0);
        variable v_wr : unsigned(19 downto 0);
        variable v_yt : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- timing class + y step, free-running from the timing ID
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => s_sd <= '1'; s_h7 <= '0';
                when 5 | 6 | 14     => s_sd <= '0'; s_h7 <= '1';
                when others         => s_sd <= '0'; s_h7 <= '0';
            end case;
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            if s_fstart = '1' then
                s_newframe <= '1';
                s_yrep     <= (others => '0');
            elsif g_avid = '1' and g_avid_q = '0' then
                -- (core_top ties field_n high, so there is no bottom-field
                -- phase to apply: both 480i fields draw the same rows)
                if s_newframe = '1' then
                    v_ar := s_aoff;
                    v_br := s_aoff;
                    v_wr := s_woff;
                    v_yt := s_ys;
                    s_newframe <= '0';
                else
                    v_ar := s_arow + s_sa;
                    v_br := s_brow + s_ca;
                    v_wr := s_wphx + s_wstep;
                    v_yt := resize(s_yf, 12) + s_ys;
                end if;
                -- virtual lines the NEXT row advances: this row start does
                -- one step, the blanking after it does the rest (never at
                -- 1080p, where the step is exactly one line)
                s_yf   <= v_yt(7 downto 0);
                s_yrep <= v_yt(10 downto 8) - 1;
                s_row0 <= s_newframe;
                s_arow <= v_ar;  s_apx <= v_ar;
                s_brow <= v_br;  s_bpx <= v_br;
                s_wphx <= v_wr;
                s_wphy <= s_woff;
            elsif g_avid = '1' then
                if s_chev = '1' and g_x >= s_cxu then
                    s_apx  <= s_apx - s_cax;     -- mirror about screen centre
                    s_bpx  <= s_bpx + s_sax;
                    s_wphy <= s_wphy - s_wsx;
                else
                    s_apx  <= s_apx + s_cax;
                    s_bpx  <= s_bpx - s_sax;
                    s_wphy <= s_wphy + s_wsx;
                end if;
            elsif s_yrep /= 0 then
                -- blanking: the extra virtual-line steps (same adders)
                s_arow <= s_arow + s_sa;
                s_brow <= s_brow + s_ca;
                s_wphx <= s_wphx + s_wstep;
                s_yrep <= s_yrep - 1;
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- PATTERN: ripple bend -> projections -> stop index -> palette.
    ------------------------------------------------------------------------
    p_pat : process(clk)
        variable v_shift : integer range 7 to 13;
        variable v_ix    : unsigned(3 downto 0);
        variable v_ua, v_ub : unsigned(4 downto 0);
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_sub   : unsigned(4 downto 0);
        variable v_tx, v_ty : signed(9 downto 0);
        variable v_top   : std_logic;
    begin
        if rising_edge(clk) then
            -- A: accumulator snapshot + ripple SHAPE (fold + one table read
            -- per axis), so W1 is left with just the two signs and one add
            a_ap <= s_apx;
            a_bp <= s_bpx;
            a_mx <= C_WAVE(to_integer(f_widx(s_wphx(19 downto 10), s_zig)));
            a_my <= C_WAVE(to_integer(f_widx(s_wphy(19 downto 10), s_zig)));
            a_sx <= s_wphx(19);
            a_sy <= s_wphy(19);
            a_x  <= g_x(10 downto 0);

            -- W1: cross-axis bend.  The two axes' displacements always enter
            -- the projection with equal weight, so they can be SUMMED BEFORE
            -- the multiply -- one multiply instead of two.
            v_tx := signed(resize(a_mx, 10));
            if a_sx = '1' then v_tx := -v_tx; end if;
            v_ty := signed(resize(a_my, 10));
            if a_sy = '1' then v_ty := -v_ty; end if;
            w1_ts <= v_tx + v_ty;
            w1_ap <= a_ap;  w1_bp <= a_bp;  w1_x <= a_x;

            -- W2: WARP depth.  The only pixel-path multiply of any size.
            -- (w1_ts is 4x the old 7-bit shape, hence >>3 where it was >>1)
            w2_ow <= resize(shift_right(w1_ts * signed('0' & s_wamp), 3), 20);
            w2_ap <= w1_ap;  w2_bp <= w1_bp;  w2_x <= w1_x;

            -- W3: bend both weaves identically so PLAID stays coherent
            w3_ap <= w2_ap + resize(w2_ow, 23);
            w3_bp <= w2_bp + resize(w2_ow, 23);
            w3_x  <= w2_x;

            -- Q: ONE barrel shift per weave yields both the stop index
            -- (bits 8..5) and the sub-stop position (bits 4..0) used by the
            -- rounding, the specular line and the outline.
            v_shift := s_qsh;
            q_a <= resize(shift_right(unsigned(w3_ap), v_shift), 9);
            q_b <= resize(shift_right(unsigned(w3_bp), v_shift), 9);
            q_x <= w3_x;

            -- ST: ROPE folds the ramp (0..7,7..0) so each ribbon is
            -- symmetric.  PLAID is an over/under WEAVE: which set is on top
            -- alternates in a checkerboard of (unfolded) stop parities.
            -- OUTLINE marks the first 1/16 of every stop of the TOP ribbon.
            if s_rope = '1' then
                v_ix := q_a(8 downto 5);
                if v_ix(3) = '1' then st_sa <= not v_ix(2 downto 0);
                else                  st_sa <= v_ix(2 downto 0); end if;
                v_ix := q_b(8 downto 5);
                if v_ix(3) = '1' then st_sb <= not v_ix(2 downto 0);
                else                  st_sb <= v_ix(2 downto 0); end if;
            else
                st_sa <= q_a(7 downto 5);
                st_sb <= q_b(7 downto 5);
            end if;
            v_ua := q_a(4 downto 0);
            v_ub := q_b(4 downto 0);
            st_ua <= v_ua;
            st_ub <= v_ub;
            v_top := s_plaid and (q_a(5) xor q_b(5));
            st_top <= v_top;
            if s_outl = '1' and ((v_top = '0' and v_ua < 2) or (v_top = '1' and v_ub < 2)) then
                st_ol <= '1';
            else
                st_ol <= '0';
            end if;
            st_x <= q_x;

            -- R1: two reads of the blended palette register file (8:1 muxes)
            ra_y <= s_ply(to_integer(st_sa));
            ra_u <= s_plu(to_integer(st_sa));
            ra_v <= s_plv(to_integer(st_sa));
            rb_y <= s_ply(to_integer(st_sb));
            rb_u <= s_plu(to_integer(st_sb));
            rb_v <= s_plv(to_integer(st_sb));
            r1_ol <= st_ol;  r1_top <= st_top;
            r1_ua <= st_ua;  r1_ub <= st_ub;  r1_x <= st_x;

            -- R2: the top ribbon of the weave; OUTLINE darkens the seam to a
            -- quarter.
            if r1_top = '1' then
                v_y := rb_y;  v_u := rb_u;  v_v := rb_v;  v_sub := r1_ub;
            else
                v_y := ra_y;  v_u := ra_u;  v_v := ra_v;  v_sub := r1_ua;
            end if;
            if r1_ol = '1' then
                r_y <= shift_right(v_y, 2);
            else
                r_y <= v_y;
            end if;
            r_u <= v_u;  r_v <= v_v;  r_sub <= v_sub;  r_x <= r1_x;
        end if;
    end process p_pat;

    ------------------------------------------------------------------------
    -- GLOSS BODY: the sub-stop position across each ribbon is its surface
    -- normal, so a linear ramp across it reads as a rounded rope and a
    -- narrow band near the light side reads as the specular highlight.
    -- This is what makes GLOSS carry weight on a flat generated field --
    -- the luma-gradient emboss below only ever fires at the seams.
    ------------------------------------------------------------------------
    p_shade : process(clk)
        variable v_s6 : signed(5 downto 0);
        variable v_ss : signed(5 downto 0);
        variable v_sd : signed(14 downto 0);
        variable v_sh : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- S1: diffuse ramp (bright toward the upper-left light) and the
            -- specular line, both scaled by K6.
            v_s6 := signed(resize(r_sub, 6));
            v_ss := v_s6 - 16;                       -- -16..15 across the rope
            -- >>4 (not >>3): the rounding tops out at +/-255 so the candy
            -- colour still reads at full GLOSS -- the specular line below is
            -- what sells the shine, an over-deep diffuse just crushes the ramp.
            v_sd := (-v_ss) * signed(resize(s_gdif, 9));
            sh_d <= resize(shift_right(v_sd, 4), 11);
            if r_sub >= 4 and r_sub <= 6 then
                sh_s <= signed('0' & s_gspc);
            else
                sh_s <= (others => '0');
            end if;

            -- S2: sum + bound
            v_sh := resize(sh_d, 12) + resize(sh_s, 12);
            if    v_sh >  700 then sh <= to_signed(700, 12);
            elsif v_sh < -450 then sh <= to_signed(-450, 12);
            else                   sh <= v_sh; end if;
        end if;
    end process p_shade;

    ------------------------------------------------------------------------
    -- GLOSS EMBOSS: line-RAM luma heightfield -> specular sheen (g_add).
    ------------------------------------------------------------------------
    p_gloss : process(clk)
        variable v_a   : signed(13 downto 0);
        variable v_lit : signed(11 downto 0);
        variable v_sh  : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- L0: line RAM (read previous line, write current).  Gated on the
            -- avid delayed to THIS stage so blanking never poisons col 0.
            -- The WRITE TRAILS THE READ BY ONE PIXEL: a same-address read and
            -- write in one cycle is UNDEFINED on iCE40 EBR (GHDL returns the
            -- old word, so no sim ever showed it).  Column x is still read
            -- before this line overwrites it.
            if s_avid_sr(8) = '1' then
                lb_rd <= lram(to_integer(r_x));
            end if;
            if lw_en = '1' then
                lram(to_integer(lw_x)) <= std_logic_vector(lw_y);
            end if;
            lw_en <= s_avid_sr(8);
            lw_x  <= r_x;
            lw_y  <= r_y;
            gpl   <= r_y;
            gpl_d <= gpl;

            -- L1: gradients (gpl=P, gpl_d=P-1 horizontal, lb_rd=P one line up)
            ggx <= signed('0' & gpl) - signed('0' & gpl_d);
            -- no vertical gradient on the frame's FIRST row: the line RAM
            -- still holds the previous frame's BOTTOM row there, which drew
            -- a noisy emboss line along the top edge.  (s_row0 only changes
            -- at a line start, ~280 clocks after this row has drained.)
            if s_row0 = '1' then
                ggy <= (others => '0');
            else
                ggy <= signed('0' & gpl) - signed('0' & lb_rd);
            end if;

            -- L2: lit = gx+gy.  Threshold + bound decided HERE (parallel, has
            -- slack) so L3 is a bare barrel shift and L4 a bare add + clamp.
            v_lit := resize(ggx, 12) + resize(ggy, 12);
            if v_lit > signed('0' & s_gspec) then spec_hi <= '1';
            else                                  spec_hi <= '0'; end if;
            if    v_lit >  255 then glit <= to_signed(255, 12);
            elsif v_lit < -255 then glit <= to_signed(-255, 12);
            else                    glit <= v_lit; end if;

            -- L3: sheen SHIFT only.  gsh = 0 -> matte.
            if s_gsh = 0 then
                v_sh := (others => '0');
            else
                v_sh := shift_left(resize(glit, 14), s_gsh - 1);
            end if;
            g_sh     <= v_sh;
            spec_hi2 <= spec_hi;

            -- L4: blown-crescent add + clamp (separate cycle from the shift).
            -- BLANKING GATE: zero outside the output avid (see p_out).
            v_a := g_sh;
            if spec_hi2 = '1' then v_a := v_a + 360; end if;
            if v_a > 511 then v_a := to_signed(511, 14);
            elsif v_a < -300 then v_a := to_signed(-300, 14); end if;
            if s_avid_sr(C_GATE_IDX) = '0' then
                g_add <= (others => '0');
            else
                g_add <= resize(v_a, 11);
            end if;
        end if;
    end process p_gloss;

    ------------------------------------------------------------------------
    -- OUTPUT: ribbon colour, delayed to meet the sheen, with the rounding
    -- folded in mid-chain (index 2) so no stage carries two adds.
    -- BLANKING GATE one stage before the output register: whenever the
    -- output avid will be low, the colour goes to neutral (64/512/512) and
    -- the sheen to 0 -- a free-running colour in blanking corrupts the
    -- encoders' colour reference.
    -- U/V are SWAPPED on the way out: the palettes are authored standard
    -- BT.601 but the hardware's u wire carries Cr and v carries Cb.
    ------------------------------------------------------------------------
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            cy_sr(0) <= r_y;      cu_sr(0) <= r_u;      cv_sr(0) <= r_v;
            cy_sr(1) <= cy_sr(0); cu_sr(1) <= cu_sr(0); cv_sr(1) <= cv_sr(0);
            -- ribbon rounding + specular line lands here
            cy_sr(2) <= f_cu10(signed(resize(cy_sr(1), 13)) + resize(sh, 13));
            cu_sr(2) <= cu_sr(1); cv_sr(2) <= cv_sr(1);
            cy_sr(3) <= cy_sr(2); cu_sr(3) <= cu_sr(2); cv_sr(3) <= cv_sr(2);
            if s_avid_sr(C_GATE_IDX) = '0' then
                cy_sr(4) <= to_unsigned(64, 10);
                cu_sr(4) <= to_unsigned(512, 10);
                cv_sr(4) <= to_unsigned(512, 10);
            else
                cy_sr(4) <= cy_sr(3); cu_sr(4) <= cu_sr(3); cv_sr(4) <= cv_sr(3);
            end if;

            o_y <= f_cleg(signed(resize(cy_sr(4), 12)) + resize(g_add, 12));
            o_u <= cv_sr(4);
            o_v <= cu_sr(4);

            -- LUMA 1-2-1 (horizontal).  The recurring ~10 px HDMI bars
            -- (CGA, Fray, ...) were reproduced on a dial here: Penny Candy's
            -- 7->0 wrap puts full white directly against true black, and Warp
            -- folds it into 1-px full-swing spikes.  Rope (no wrap edge) or
            -- this low-pass on the DIAG build cleared them on hardware.  A
            -- full black<->white step becomes a 2-px ramp -- reads as AA at
            -- HD.  Chroma untouched.  Blanking is re-gated here: the taps
            -- mix the neutral blanking value into the edge pixels.
            f_y1 <= o_y;  f_y2 <= f_y1;
            f_u1 <= o_u;  f_v1 <= o_v;
            if s_avid_sr(C_LATENCY - 2) = '0' then
                f_out_y <= to_unsigned(64, 10);
                f_out_u <= to_unsigned(512, 10);
                f_out_v <= to_unsigned(512, 10);
            else
                f_out_y <= resize(shift_right(resize(f_y2, 12) + shift_left(resize(f_y1, 12), 1)
                                              + resize(o_y, 12) + 2, 2), 10);
                f_out_u <= f_u1;
                f_out_v <= f_v1;
            end if;
        end if;
    end process p_out;

    ------------------------------------------------------------------------
    -- sync delay + output.  avid is the GENERATED one; syncs pass through.
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= g_avid          & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(f_out_y);
    data_out.u       <= std_logic_vector(f_out_u);
    data_out.v       <= std_logic_vector(f_out_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture ribbon;
