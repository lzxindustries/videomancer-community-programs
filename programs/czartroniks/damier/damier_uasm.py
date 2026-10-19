#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of Videomancer Community Programs.
# See LICENSE file in the repository root for full license text.
#
"""DAMIER microcode assembler (ported from CUBIST's, 16-bit datapath).

One small engine runs the per-field work: a single adder ALU, a serial
shifter, a radix-4 Booth serial multiplier, a quarter-sine ROM unit (512
entries a quarter), a block-RAM register file (two copies for dual read) and
the program in block RAM.

Instruction: 32 bits
  [31:27] op   [26:20] dst   [19:13] srcA   [12:6] srcB   [5:0] i6

Immediates overlap the fields an op does not use:
  LDI            imm14 = [13:0]  (sign-extended)
  AND            imm13 = [12:0]  (zero-extended; srcB unused)
  JMP JNZ        target = [9:0]
  JLT JGE        target = [23:20] & [5:0]   (a branch writes no register)
  SHR SHL        count [3:0]
  MRD            shift [4:0]  (0..16: dst = product >> shift, low 16 bits)
  SIN            [0] = cos
  PRT            port [5:0], value = A, aux (address) = B
  CTL            input [3:0]
"""

OPS = {
    'NOP': 0, 'MOV': 1, 'ADD': 2, 'SUB': 3, 'SHR': 4, 'SHL': 5,
    'MUL': 10,   # start A * B (16 x 16 signed)
    'MRD': 11,   # dst = product >> imm (stalls until the multiply is idle)
    'SIN': 14,   # dst = sin(A) Q15 (cos when imm=1), 2048 steps per turn
    'LDI': 15,   # dst = imm14
    'PRT': 17,   # port imm <= A (aux B)
    'JMP': 18, 'JNZ': 19, 'JLT': 20, 'JGE': 21,
    'END': 24,   # halt until the next field
    'CTL': 25,   # dst = control input imm
    'JR':  26,   # pc = A
    'AND': 27,   # dst = A and imm13
}

PORTS = {
    'FA': 0, 'FB': 1, 'CU': 2, 'CV': 3, 'RF': 4, 'RQ': 5,
    'T8': 7, 'CWA': 8, 'CWB': 9, 'TW': 10,
    'TSEL': 13, 'WTAB': 14, 'FDONE': 16, 'BDONE': 17,
    'NA': 19, 'NB': 20, 'Y0': 21,
}

CTLS = {
    'k2': 0, 'k3': 1, 'k4': 2, 'vt': 3, 'k5': 4, 'flags': 5,
    'W2': 6, 'H2': 7, 'R9': 8,
}

# flags input bits (FL_720 / FL_SD: the resolution class, v0.4.8)
FL_FOLD, FL_ZZ, FL_SQ, FL_SCRL, FL_720, FL_SD = 1, 2, 4, 8, 16, 32


class Asm:
    def __init__(self):
        self.code = []
        self.labels = {}
        self.fixups = []
        self.regs = {}
        self._n = 0

    # register allocation by name
    def r(self, name):
        if name not in self.regs:
            assert len(self.regs) < 128, 'register file full'
            self.regs[name] = len(self.regs)
        return self.regs[name]

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def _lbl(self):
        self._n += 1
        return f'__m{self._n}'

    def emit(self, op, dst=None, a=None, b=None, imm=0):
        R = lambda x: 0 if x is None else (self.r(x) if isinstance(x, str) else x)
        zero = self.r('zero')
        # macros: the ALU keeps one adder
        if op == 'NEG':
            return self.emit('SUB', dst, 'zero', a)
        if op == 'ABS':
            l1, l2 = self._lbl(), self._lbl()
            self.emit('JLT', a=a, b='zero', imm=l1)
            self.emit('MOV', dst, a)
            self.emit('JMP', imm=l2)
            self.label(l1)
            self.emit('SUB', dst, 'zero', a)
            self.label(l2)
            return
        if op == 'SAT':                     # dst = clamp(a, lo=b, hi=imm-register)
            lo, hi = b, imm
            l1, l2, l3 = self._lbl(), self._lbl(), self._lbl()
            self.emit('JLT', a=a, b=lo, imm=l1)
            self.emit('JLT', a=hi, b=a, imm=l2)
            self.emit('MOV', dst, a)
            self.emit('JMP', imm=l3)
            self.label(l1)
            self.emit('MOV', dst, lo)
            self.emit('JMP', imm=l3)
            self.label(l2)
            self.emit('MOV', dst, hi)
            self.label(l3)
            return
        assert op in OPS, f'{op} is not implemented'
        if op in ('SHR', 'SHL'):
            assert 0 <= imm <= 15, imm
        if op == 'MRD':
            assert 0 <= imm <= 16, imm
        if op == 'AND':
            assert 0 <= imm < 8192, imm
        if op == 'LDI' and not isinstance(imm, str):
            assert -8192 <= imm < 8192, imm
        if op == 'PRT' and isinstance(imm, str):
            imm = PORTS[imm]
        if op == 'CTL' and isinstance(imm, str):
            imm = CTLS[imm]
        if isinstance(imm, str):
            self.fixups.append((len(self.code), imm))
            imm = 0
        self.code.append([OPS[op], R(dst), R(a), R(b), imm])
        return len(self.code) - 1

    def resolve(self):
        for idx, name in self.fixups:
            assert name in self.labels, f'undefined label {name}'
            self.code[idx][4] = self.labels[name]
        return self.code

    def words(self):
        inv = {v: k for k, v in OPS.items()}
        out = []
        for op, dst, a, b, imm in self.resolve():
            name = inv[op]
            w = op << 27
            if name in ('JLT', 'JGE'):
                assert 0 <= imm < 1024
                w |= ((imm >> 6) & 0xF) << 20 | a << 13 | b << 6 | (imm & 0x3F)
            elif name in ('JMP', 'JNZ'):
                assert 0 <= imm < 1024
                w |= a << 13 | imm
            elif name == 'LDI':
                w |= dst << 20 | (imm & 0x3FFF)
            elif name == 'AND':
                w |= dst << 20 | a << 13 | (imm & 0x1FFF)
            else:
                assert 0 <= imm < 64, (name, imm)
                w |= dst << 20 | a << 13 | b << 6 | imm
            out.append(w)
        return out


def emit_vhdl(words, name='C_UCODE', depth=512):
    assert len(words) <= depth, f'program is {len(words)} words, ROM is {depth}'
    words = list(words) + [0] * (depth - len(words))
    rows, row = [], []
    for i, w in enumerate(words):
        row.append(f'x"{w:08X}"')
        if len(row) == 6 or i == depth - 1:
            rows.append('        ' + ', '.join(row))
            row = []
    return (f'    type t_ucode is array (0 to {depth-1}) of std_logic_vector(31 downto 0);\n'
            f'    constant {name} : t_ucode := (\n' + ',\n'.join(rows) + ' );')
