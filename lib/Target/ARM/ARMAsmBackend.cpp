//===-- ARMAsmBackend.cpp - ARM Assembler Backend -------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "ARM.h"
#include "ARMAddressingModes.h"
#include "ARMFixupKinds.h"
#include "llvm/ADT/Twine.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectFormat.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSectionMachO.h"
#include "llvm/Object/MachOFormat.h"
#include "llvm/Support/ELF.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetAsmBackend.h"
#include "llvm/Target/TargetRegistry.h"
using namespace llvm;

namespace {
class ARMAsmBackend : public TargetAsmBackend {
  bool isThumbMode;  // Currently emitting Thumb code.
public:
  ARMAsmBackend(const Target &T) : TargetAsmBackend(), isThumbMode(false) {}

  bool MayNeedRelaxation(const MCInst &Inst) const;

  void RelaxInstruction(const MCInst &Inst, MCInst &Res) const;

  bool WriteNopData(uint64_t Count, MCObjectWriter *OW) const;

  void HandleAssemblerFlag(MCAssemblerFlag Flag) {
    switch (Flag) {
    default: break;
    case MCAF_Code16:
      setIsThumb(true);
      break;
    case MCAF_Code32:
      setIsThumb(false);
      break;
    }
  }

  unsigned getPointerSize() const { return 4; }
  bool isThumb() const { return isThumbMode; }
  void setIsThumb(bool it) { isThumbMode = it; }
};
} // end anonymous namespace

bool ARMAsmBackend::MayNeedRelaxation(const MCInst &Inst) const {
  // FIXME: Thumb targets, different move constant targets..
  return false;
}

void ARMAsmBackend::RelaxInstruction(const MCInst &Inst, MCInst &Res) const {
  assert(0 && "ARMAsmBackend::RelaxInstruction() unimplemented");
  return;
}

bool ARMAsmBackend::WriteNopData(uint64_t Count, MCObjectWriter *OW) const {
  if (isThumb()) {
    assert (((Count & 1) == 0) && "Unaligned Nop data fragment!");
    // FIXME: 0xbf00 is the ARMv7 value. For v6 and before, we'll need to
    // use 0x46c0 (which is a 'mov r8, r8' insn).
    Count /= 2;
    for (uint64_t i = 0; i != Count; ++i)
      OW->Write16(0xbf00);
    return true;
  }
  // ARM mode
  Count /= 4;
  for (uint64_t i = 0; i != Count; ++i)
    OW->Write32(0xe1a00000);
  return true;
}

static unsigned adjustFixupValue(unsigned Kind, uint64_t Value) {
  switch (Kind) {
  default:
    llvm_unreachable("Unknown fixup kind!");
  case FK_Data_4:
    return Value;
  case ARM::fixup_arm_movt_hi16:
  case ARM::fixup_arm_movw_lo16: {
    unsigned Hi4 = (Value & 0xF000) >> 12;
    unsigned Lo12 = Value & 0x0FFF;
    // inst{19-16} = Hi4;
    // inst{11-0} = Lo12;
    Value = (Hi4 << 16) | (Lo12);
    return Value;
  }
  case ARM::fixup_arm_ldst_pcrel_12:
    // ARM PC-relative values are offset by 8.
    Value -= 4;
    // FALLTHROUGH
  case ARM::fixup_t2_ldst_pcrel_12: {
    // Offset by 4, adjusted by two due to the half-word ordering of thumb.
    Value -= 4;
    bool isAdd = true;
    if ((int64_t)Value < 0) {
      Value = -Value;
      isAdd = false;
    }
    assert ((Value < 4096) && "Out of range pc-relative fixup value!");
    Value |= isAdd << 23;
    
    // Same addressing mode as fixup_arm_pcrel_10,
    // but with 16-bit halfwords swapped.
    if (Kind == ARM::fixup_t2_ldst_pcrel_12) {
      uint64_t swapped = (Value & 0xFFFF0000) >> 16;
      swapped |= (Value & 0x0000FFFF) << 16;
      return swapped;
    }
    
    return Value;
  }
  case ARM::fixup_arm_adr_pcrel_12: {
    // ARM PC-relative values are offset by 8.
    Value -= 8;
    unsigned opc = 4; // bits {24-21}. Default to add: 0b0100
    if ((int64_t)Value < 0) {
      Value = -Value;
      opc = 2; // 0b0010
    }
    assert(ARM_AM::getSOImmVal(Value) != -1 &&
           "Out of range pc-relative fixup value!");
    // Encode the immediate and shift the opcode into place.
    return ARM_AM::getSOImmVal(Value) | (opc << 21);
  }
  case ARM::fixup_arm_branch:
    // These values don't encode the low two bits since they're always zero.
    // Offset by 8 just as above.
    return 0xffffff & ((Value - 8) >> 2);
  case ARM::fixup_t2_branch: {
    Value = Value - 6;
    Value >>= 1; // Low bit is not encoded.
    
    uint64_t out = 0;
    out |= (Value & 0x80000) << 7; // S bit
    out |= (Value & 0x40000) >> 7; // J2 bit
    out |= (Value & 0x20000) >> 4; // J1 bit
    out |= (Value & 0x1F800) << 5; // imm6 field
    out |= (Value & 0x007FF);      // imm11 field
    
    uint64_t swapped = (out & 0xFFFF0000) >> 16;
    swapped |= (out & 0x0000FFFF) << 16;
    return swapped;
  }
  case ARM::fixup_arm_thumb_bl: {
    // The value doesn't encode the low bit (always zero) and is offset by
    // four. The value is encoded into disjoint bit positions in the destination
    // opcode. x = unchanged, I = immediate value bit, S = sign extension bit
    // 
    //   BL:  xxxxxSIIIIIIIIII xxxxxIIIIIIIIIII
    // 
    // Note that the halfwords are stored high first, low second; so we need
    // to transpose the fixup value here to map properly.
    unsigned isNeg = (int64_t(Value) < 0) ? 1 : 0;
    uint32_t Binary = 0;
    Value = 0x3fffff & ((Value - 4) >> 1);
    Binary  = (Value & 0x7ff) << 16;    // Low imm11 value.
    Binary |= (Value & 0x1ffc00) >> 11; // High imm10 value.
    Binary |= isNeg << 10;              // Sign bit.
    return Binary;
  }
  case ARM::fixup_arm_thumb_blx: {
    // The value doesn't encode the low two bits (always zero) and is offset by
    // four (see fixup_arm_thumb_cp). The value is encoded into disjoint bit
    // positions in the destination opcode. x = unchanged, I = immediate value
    // bit, S = sign extension bit, 0 = zero.
    // 
    //   BLX: xxxxxSIIIIIIIIII xxxxxIIIIIIIIII0
    // 
    // Note that the halfwords are stored high first, low second; so we need
    // to transpose the fixup value here to map properly.
    unsigned isNeg = (int64_t(Value) < 0) ? 1 : 0;
    uint32_t Binary = 0;
    Value = 0xfffff & ((Value - 2) >> 2);
    Binary  = (Value & 0x3ff) << 17;    // Low imm10L value.
    Binary |= (Value & 0xffc00) >> 10;  // High imm10H value.
    Binary |= isNeg << 10;              // Sign bit.
    return Binary;
  }
  case ARM::fixup_arm_thumb_cp:
    // Offset by 4, and don't encode the low two bits. Two bytes of that
    // 'off by 4' is implicitly handled by the half-word ordering of the
    // Thumb encoding, so we only need to adjust by 2 here.
    return ((Value - 2) >> 2) & 0xff;
  case ARM::fixup_arm_thumb_cb: {
    // Offset by 4 and don't encode the lower bit, which is always 0.
    uint32_t Binary = (Value - 4) >> 1;
    return ((Binary & 0x20) << 9) | ((Binary & 0x1f) << 3);
  }
  case ARM::fixup_arm_pcrel_10:
    Value = Value - 6; // ARM fixups offset by an additional word and don't
                       // need to adjust for the half-word ordering.
    // Fall through.
  case ARM::fixup_t2_pcrel_10: {
    // Offset by 4, adjusted by two due to the half-word ordering of thumb.
    Value = Value - 2;
    bool isAdd = true;
    if ((int64_t)Value < 0) {
      Value = -Value;
      isAdd = false;
    }
    // These values don't encode the low two bits since they're always zero.
    Value >>= 2;
    assert ((Value < 256) && "Out of range pc-relative fixup value!");
    Value |= isAdd << 23;

    // Same addressing mode as fixup_arm_pcrel_10,
    // but with 16-bit halfwords swapped.
    if (Kind == ARM::fixup_t2_pcrel_10) {
      uint64_t swapped = (Value & 0xFFFF0000) >> 16;
      swapped |= (Value & 0x0000FFFF) << 16;
      return swapped;
    }

    return Value;
  }
  }
}

namespace {

// FIXME: This should be in a separate file.
// ELF is an ELF of course...
class ELFARMAsmBackend : public ARMAsmBackend {
  MCELFObjectFormat Format;

public:
  Triple::OSType OSType;
  ELFARMAsmBackend(const Target &T, Triple::OSType _OSType)
    : ARMAsmBackend(T), OSType(_OSType) {
    HasScatteredSymbols = true;
  }

  virtual const MCObjectFormat &getObjectFormat() const {
    return Format;
  }

  void ApplyFixup(const MCFixup &Fixup, char *Data, unsigned DataSize,
                  uint64_t Value) const;

  MCObjectWriter *createObjectWriter(raw_ostream &OS) const {
    return createELFObjectWriter(OS, /*Is64Bit=*/false,
                                 OSType, ELF::EM_ARM,
                                 /*IsLittleEndian=*/true,
                                 /*HasRelocationAddend=*/false);
  }
};

// FIXME: Raise this to share code between Darwin and ELF.
void ELFARMAsmBackend::ApplyFixup(const MCFixup &Fixup, char *Data,
                                  unsigned DataSize, uint64_t Value) const {
  unsigned NumBytes = 4;        // FIXME: 2 for Thumb
  Value = adjustFixupValue(Fixup.getKind(), Value);
  if (!Value) return;           // Doesn't change encoding.

  unsigned Offset = Fixup.getOffset();
  assert(Offset % NumBytes == 0 && "Offset mod NumBytes is nonzero!");

  // For each byte of the fragment that the fixup touches, mask in the bits from
  // the fixup value. The Value has been "split up" into the appropriate
  // bitfields above.
  for (unsigned i = 0; i != NumBytes; ++i)
    Data[Offset + i] |= uint8_t((Value >> (i * 8)) & 0xff);
}

// FIXME: This should be in a separate file.
class DarwinARMAsmBackend : public ARMAsmBackend {
  MCMachOObjectFormat Format;
public:
  DarwinARMAsmBackend(const Target &T) : ARMAsmBackend(T) {
    HasScatteredSymbols = true;
  }

  virtual const MCObjectFormat &getObjectFormat() const {
    return Format;
  }

  void ApplyFixup(const MCFixup &Fixup, char *Data, unsigned DataSize,
                  uint64_t Value) const;

  MCObjectWriter *createObjectWriter(raw_ostream &OS) const {
    // FIXME: Subtarget info should be derived. Force v7 for now.
    return createMachObjectWriter(OS, /*Is64Bit=*/false,
                                  object::mach::CTM_ARM,
                                  object::mach::CSARM_V7,
                                  /*IsLittleEndian=*/true);
  }

  virtual bool doesSectionRequireSymbols(const MCSection &Section) const {
    return false;
  }
};

/// getFixupKindNumBytes - The number of bytes the fixup may change.
static unsigned getFixupKindNumBytes(unsigned Kind) {
  switch (Kind) {
  default:
    llvm_unreachable("Unknown fixup kind!");

  case ARM::fixup_arm_thumb_cp:
    return 1;

  case ARM::fixup_arm_thumb_cb:
    return 2;

  case ARM::fixup_arm_ldst_pcrel_12:
  case ARM::fixup_arm_pcrel_10:
  case ARM::fixup_arm_adr_pcrel_12:
  case ARM::fixup_arm_branch:
    return 3;

  case FK_Data_4:
  case ARM::fixup_t2_ldst_pcrel_12:
  case ARM::fixup_t2_branch:
  case ARM::fixup_t2_pcrel_10:
  case ARM::fixup_arm_thumb_bl:
  case ARM::fixup_arm_thumb_blx:
    return 4;
  }
}

void DarwinARMAsmBackend::ApplyFixup(const MCFixup &Fixup, char *Data,
                                     unsigned DataSize, uint64_t Value) const {
  unsigned NumBytes = getFixupKindNumBytes(Fixup.getKind());
  Value = adjustFixupValue(Fixup.getKind(), Value);
  if (!Value) return;           // Doesn't change encoding.

  unsigned Offset = Fixup.getOffset();
  assert(Offset + NumBytes <= DataSize && "Invalid fixup offset!");

  // For each byte of the fragment that the fixup touches, mask in the
  // bits from the fixup value.
  for (unsigned i = 0; i != NumBytes; ++i)
    Data[Offset + i] |= uint8_t((Value >> (i * 8)) & 0xff);
}

} // end anonymous namespace

TargetAsmBackend *llvm::createARMAsmBackend(const Target &T,
                                            const std::string &TT) {
  switch (Triple(TT).getOS()) {
  case Triple::Darwin:
    return new DarwinARMAsmBackend(T);
  case Triple::MinGW32:
  case Triple::Cygwin:
  case Triple::Win32:
    assert(0 && "Windows not supported on ARM");
  default:
    return new ELFARMAsmBackend(T, Triple(TT).getOS());
  }
}
