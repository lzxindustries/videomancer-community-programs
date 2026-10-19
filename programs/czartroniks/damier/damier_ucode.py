#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of Videomancer Community Programs.
# See LICENSE file in the repository root for full license text.
#
"""DAMIER microcode: the per-field frame sequencer and the table builder.

Runs from the field anchor:
  1. knob glides, motion phases, zoom, lattice rotation, wave phases, drift,
     the Resonance ring frequency and phase (P12)         -> PRT to the fabric
  2. FDONE: the line seeder may seed the field's first line
  3. wave tables A and B (back bank): 128 entries {amp shape, slope}
  4. BDONE + END: the banks swap at the next anchor

Formats: angles 65536 per turn; sines Q15; lattice offsets 1/256 square.
Persistent registers (never re-initialised): the four glides and their
deadband holds, T, th, sdr, sfr.  The register file powers up at zero.
"""
from damier_uasm import Asm, FL_FOLD, FL_ZZ, FL_SQ, FL_SCRL, FL_720, FL_SD

K_LA = 630      # 16 * 256 / 6.5 squares
K_LB = 390      # 16 * 256 / 10.5 squares

def build():
    A = Asm()
    e = A.emit

    def ldc(dst, value):
        """load any 16-bit constant (LDI is 14-bit)"""
        if -8192 <= value < 8192:
            e('LDI', dst, imm=value)
        else:
            assert value % 4 == 0 and -32768 <= value < 32768
            e('LDI', dst, imm=value // 4)
            e('SHL', dst, dst, imm=2)

    def mulr(dst, a, b, sh):
        e('MUL', a=a, b=b)
        e('MRD', dst, imm=sh)

    def sini(dst, a, cos=0):
        """sine (cos) interpolated between the 512-entry ROM's points, 32
        angle units apart: the bare ROM steps 1/2048 turn, a 2-3 px jump at
        1000 px from a rotation centre"""
        e('SIN', 'sn0', a, imm=cos)
        e('ADD', 'sna', a, 'c32')
        e('SIN', 'sn1', 'sna', imm=cos)
        e('SUB', 'sn1', 'sn1', 'sn0')
        e('AND', 'sna', a, imm=31)
        mulr('sn1', 'sn1', 'sna', 5)
        e('ADD', dst, 'sn0', 'sn1')

    def glide(g, h, ctl, out):
        """deadband then glide: the hold h follows raw only when they differ
        by 5+ counts (pot noise of +-2 must not reach the picture -- one
        count moved Resonance's rings by up to a quarter ring); then
        g += (h << 4 - g) >> 2 ; out = g >> 4   (10.4, per field)"""
        keep = A._lbl()
        e('CTL', 't0', imm=ctl)
        e('SUB', 't1', 't0', h)
        e('ABS', 't1', 't1')
        e('LDI', 't2', imm=5)
        e('JLT', a='t1', b='t2', imm=keep)
        e('MOV', h, 't0')
        A.label(keep)
        e('SHL', 't0', h, imm=4)
        e('SUB', 't0', 't0', g)
        e('SHR', 't0', 't0', imm=2)
        e('ADD', g, g, 't0')
        e('SHR', out, g, imm=4)

    # ---------------- constants (rebuilt every field) ----------------
    e('LDI', 'zero', imm=0)
    e('LDI', 'one', imm=1)
    e('LDI', 'c32', imm=32)
    e('LDI', 'c128', imm=128)
    e('LDI', 'c64', imm=64)
    e('LDI', 'c255', imm=255)
    e('LDI', 'c256', imm=256)
    e('LDI', 'c512', imm=512)
    e('LDI', 'c1024', imm=1024)
    ldc('c16384', 16384)
    e('ADD', 'c32767', 'c16384', 'c16384')
    e('SUB', 'c32767', 'c32767', 'one')

    # ---------------- knob glides ----------------
    glide('g2', 'h2', 'k2', 'k2')
    glide('g3', 'h3', 'k3', 'k3')
    glide('g4', 'h4', 'k4', 'k4')
    glide('gv', 'hv', 'vt', 'vt')

    # ---------------- time ----------------
    e('CTL', 'k5', imm='k5')
    mulr('dt', 'k5', 'k5', 12)                  # 0..255, square law
    e('ADD', 'T', 'T', 'dt')                    # master time (persistent)
    e('SHR', 'twa', 'T', imm=3)
    e('AND', 'twa', 'twa', imm=8191)            # wave A travels T/8 per field
    e('PRT', a='twa', imm='TW')                 # (the layout's wave sample)
    e('ADD', 't0', 'T', 'T')
    e('ADD', 't0', 't0', 'T')
    e('SHR', 't0', 't0', imm=4)
    e('AND', 'twb', 't0', imm=4095)             # wave B 3T/16
    e('SHL', 't0', 'T', imm=2)
    e('PRT', a='t0', imm='T8')                  # layout time byte = bits 15:8
    e('CTL', 'fl', imm='flags')
    e('AND', 't0', 'fl', imm=FL_FOLD)
    e('JNZ', a='t0', imm='spin')
    e('JMP', imm='nospin')
    A.label('spin')                             # any mirror: the lattice turns
    e('SHR', 't0', 'dt', imm=1)
    e('ADD', 'th', 'th', 't0')
    A.label('nospin')

    # ---------------- zoom: cell = 8 * 2^(5 k2 / 1024) px ----------------
    # s = 2^(13 - 5k/1024) = mant * 2^(4 - n), mant ~ 512 * 2^-f (linear);
    # from the glide's 10.4 value, 256 mantissa steps an octave
    e('LDI', 't0', imm=5)
    e('MUL', a='g2', b='t0')                    # 5 g2: 0 .. 81840
    e('MRD', 'n', imm=14)                       # octave 0..4
    e('MRD', 't0', imm=6)
    e('AND', 't0', 't0', imm=255)
    e('SUB', 's', 'c512', 't0')                 # 512 .. 257
    e('MOV', 'mant', 's')
    e('LDI', 't0', imm=4)
    e('SUB', 'cnt', 't0', 'n')
    A.label('zl')
    e('JGE', a='zero', b='cnt', imm='zd')
    e('SHL', 's', 's', imm=1)
    e('SUB', 'cnt', 'cnt', 'one')
    e('JMP', imm='zl')
    A.label('zd')                               # s = 256 .. 8192
    # Resolution scaling (v0.4.8): sizes are set on a virtual 1080-line
    # canvas, so s (the lattice step per PIXEL) grows by the pixel's width
    # in virtual units: x1.5 at 720p, x2 at SD (720 px = a 1440-unit 4:3
    # window).  Squares keep their size against the screen height.  mant
    # and n stay unscaled: the ring frequency below is per cell, so the
    # ring period scales with the cells.
    e('AND', 't0', 'fl', imm=FL_SD)
    e('JNZ', a='t0', imm='zsd')
    e('AND', 't0', 'fl', imm=FL_720)
    e('JNZ', a='t0', imm='z72')
    e('JMP', imm='zsc')
    A.label('z72')
    e('SHR', 't0', 's', imm=1)
    e('ADD', 's', 's', 't0')                    # 384 .. 12288
    e('JMP', imm='zsc')
    A.label('zsd')
    e('SHL', 's', 's', imm=1)                   # 512 .. 16384
    A.label('zsc')

    # ---------------- lattice rotation: K4 angle + spin ----------------
    e('SHL', 'al', 'g4', imm=2)                 # 10.4 glide: 1/16384 turn steps
    e('ADD', 'al', 'al', 'th')
    sini('t1', 'al', cos=1)
    mulr('fa', 's', 't1', 15)
    e('PRT', a='fa', imm='FA')
    e('NEG', 't0', 'fa')
    e('PRT', a='t0', imm='NA')
    sini('t1', 'al')
    mulr('fb', 's', 't1', 15)
    e('PRT', a='fb', imm='FB')
    e('NEG', 't0', 'fb')
    e('PRT', a='t0', imm='NB')

    # ---------------- wave amplitudes and displacement direction ----------
    e('SHR', 'ampA', 'k3', imm=2)
    e('SHR', 't0', 'k3', imm=4)
    e('ADD', 'ampA', 'ampA', 't0')
    e('SHR', 't0', 'k3', imm=5)
    e('ADD', 'ampA', 'ampA', 't0')              # ~0.34 k3 (1/256 square)
    e('SUB', 'ampB', 'k3', 'c512')
    e('JLT', a='ampB', b='zero', imm='nob')
    e('SHR', 'ampB', 'ampB', imm=1)             # crossing wave past mid-K3
    e('JMP', imm='bok')
    A.label('nob')
    e('MOV', 'ampB', 'zero')
    A.label('bok')
    # ---------------- drift: diagonally along the lattice ----------------
    e('AND', 't0', 'fl', imm=FL_SCRL)
    e('JNZ', a='t0', imm='scr')
    e('JMP', imm='scd')
    A.label('scr')                              # drift += dt / 32 (1/256 square)
    e('SHL', 't0', 'dt', imm=3)
    e('ADD', 'sfr', 'sfr', 't0')
    e('SHR', 't0', 'sfr', imm=8)
    e('ADD', 'sdr', 'sdr', 't0')
    e('AND', 'sfr', 'sfr', imm=255)
    A.label('scd')

    # ---------------- Resonance (P12) ----------------
    # Every cell fills with concentric rings of its layout's own metric m,
    # XORed over the cell.  The ring PERIOD is set in pixels, so the look is
    # the same at any Scale: 1/period = (2^(8 r/1024) - 1) / 256 px with
    # r = slider / 2, from infinity at 0 (no rings: the picture is
    # untouched) through ~85 px (50%) to ~17 px at 100%.  In cycles per
    # half cell x 16: F16 = cell_px Y / 4096, Y = 128 (2^(s/128) - 1),
    # cell_px = 65536 / zoom step.  The rings stand still (phase 0): they
    # change only with the slider.  (A moving phase also flips whole cells
    # at low frequency -- r = -Q everywhere.)
    def shr_var(dst, cnt):
        lp, dn = A._lbl(), A._lbl()
        A.label(lp)
        e('JGE', a='zero', b=cnt, imm=dn)
        e('SHR', dst, dst, imm=1)
        e('SUB', cnt, cnt, 'one')
        e('JMP', imm=lp)
        A.label(dn)

    # Tuned range (v0.4.3): the slider's full travel spans what was its
    # lower half -- past that the rings got too fine -- so the ring maths
    # sees half the glided slider: gr = gv / 2, vr = vt / 2
    e('SHR', 'gr', 'gv', imm=1)
    e('SHR', 'vr', 'vt', imm=1)
    # Y = (2048 + gr % 2048) << (gr / 2048) >> 4  - 128   (0 .. 1792),
    # from the slider glide's 10.4 value, so Y moves in sub-count steps
    e('SHR', 'cnt', 'gr', imm=11)               # octave 0..3
    e('AND', 't0', 'gr', imm=2047)
    e('LDI', 't1', imm=2048)
    e('ADD', 'ry', 't0', 't1')
    e('LDI', 't0', imm=4)
    e('SUB', 'cnt', 'cnt', 't0')                # shift = octave - 4
    e('JLT', a='cnt', b='zero', imm='ryr')
    A.label('ryl')
    e('JGE', a='zero', b='cnt', imm='ryd')
    e('SHL', 'ry', 'ry', imm=1)
    e('SUB', 'cnt', 'cnt', 'one')
    e('JMP', imm='ryl')
    A.label('ryr')
    e('NEG', 'cnt', 'cnt')
    shr_var('ry', 'cnt')
    A.label('ryd')
    e('SUB', 'ry', 'ry', 'c128')
    # 65536 / mant by Newton from 384 - mant/2 (mant = 260 .. 512)
    e('SHR', 't0', 'mant', imm=1)
    e('LDI', 'rcp', imm=384)
    e('SUB', 'rcp', 'rcp', 't0')
    for _ in range(2):
        mulr('t0', 'mant', 'rcp', 2)            # mant rcp / 4  (~16384)
        e('SUB', 't1', 'c16384', 't0')
        e('ADD', 't1', 't1', 'c16384')          # 32768 - mant rcp / 4
        mulr('rcp', 'rcp', 't1', 14)
    mulr('rf', 'rcp', 'ry', 12)                 # F16 at n = 4 ...
    e('LDI', 't0', imm=4)
    e('SUB', 'cnt', 't0', 'n')
    shr_var('rf', 'cnt')                        # ... >> (4 - n): cell_px
    # floor: rings per cell grow with the slider whatever the Scale, so small
    # squares answer from ~6% on and keep changing to 100%: s/8 to 25%, then
    # 24 + s/32 (3.4 rings per half cell at 100%)
    e('SHR', 't0', 'vr', imm=3)
    e('JLT', a='vr', b='c256', imm='rfc')
    e('SHR', 't0', 'vr', imm=5)
    e('LDI', 't1', imm=24)
    e('ADD', 't0', 't0', 't1')
    A.label('rfc')
    e('JGE', a='rf', b='t0', imm='rfm')
    e('MOV', 'rf', 't0')
    A.label('rfm')
    e('PRT', a='rf', imm='RF')
    # the rings stand still: they change only with the slider (and ride
    # the cells when the board itself moves); the ring phase is fixed at 0
    e('PRT', a='zero', imm='RQ')

    # ---------------- lattice origin: the drift, and the wave phases there
    e('PRT', a='sdr', imm='CU')
    e('SUB', 't0', 'sdr', 'twa')
    e('PRT', a='t0', imm='CWA')
    e('PRT', a='sdr', imm='CV')
    e('SUB', 't0', 'sdr', 'twb')
    e('PRT', a='t0', imm='CWB')
    # the field's top line, Q.4 Y units: -H2 (frame lines) x R9 (Q.9 units
    # per frame line); -16 H2 at 1080 and 720p
    e('CTL', 't0', imm='H2')
    e('CTL', 't1', imm='R9')
    mulr('t0', 't0', 't1', 5)
    e('NEG', 't0', 't0')
    e('PRT', a='t0', imm='Y0')
    e('PRT', imm='FDONE')

    # ---------------- wave tables ----------------
    # A runs along U, B along V; each entry k-1 (k = 0..128) is a scalar
    # {amp shape(k-1) << 6 | slope}; the pixel path adds it to the lattice
    # axis that Bend / Squeeze selects
    e('LDI', 'c129', imm=129)
    e('MOV', 'wsel', 'zero')
    A.label('wv')
    e('PRT', a='wsel', imm='TSEL')
    e('MOV', 'wa', 'ampA')
    e('JNZ', a='wsel', imm='wvb')
    e('JMP', imm='wvd')
    A.label('wvb')
    e('MOV', 'wa', 'ampB')
    A.label('wvd')
    e('MOV', 'k', 'zero')
    A.label('wk')
    e('AND', 't0', 'fl', imm=FL_ZZ)
    e('JNZ', a='t0', imm='tri')
    e('SHL', 't0', 'k', imm=9)
    e('SIN', 'sh', 't0')
    e('JMP', imm='shd')
    A.label('tri')                              # zigzag: (|k-64| - 32) * 1023
    e('AND', 't0', 'k', imm=127)
    e('SUB', 't0', 't0', 'c64')
    e('ABS', 't0', 't0')
    e('LDI', 't1', imm=32)
    e('SUB', 't0', 't0', 't1')
    e('SHL', 't1', 't0', imm=10)
    e('SUB', 'sh', 't1', 't0')
    A.label('shd')
    mulr('du', 'wa', 'sh', 15)
    e('JGE', a='zero', b='k', imm='nx')
    e('SUB', 't0', 'du', 'pu')
    e('AND', 't0', 't0', imm=63)
    e('SHL', 't1', 'pu', imm=6)
    e('ADD', 't1', 't1', 't0')
    e('SUB', 't0', 'k', 'one')
    e('PRT', a='t1', b='t0', imm='WTAB')
    A.label('nx')
    e('MOV', 'pu', 'du')
    e('ADD', 'k', 'k', 'one')
    e('JLT', a='k', b='c129', imm='wk')
    e('ADD', 'wsel', 'wsel', 'one')
    e('LDI', 't0', imm=2)
    e('JLT', a='wsel', b='t0', imm='wv')

    e('PRT', imm='BDONE')
    e('END')
    return A


if __name__ == '__main__':
    a = build()
    w = a.words()
    print(len(w), 'words,', len(a.regs), 'registers')
