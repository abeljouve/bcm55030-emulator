# Fidelity and known divergences

This emulator is accurate enough to boot and exercise firmware, but it is **not
a cycle-accurate or analog-accurate model** of the BCM55030. Use it for
functional work — not for timing-sensitive or PHY-level validation.

## CPU core (ARC700 / ARCompact)

The ARC700 core is modelled faithfully for this SoC's configuration, including
where it diverges from a textbook ARC700:

- **No `rtie`.** This SoC's core does not implement the `rtie` instruction. The
  emulator decodes it (so the disassembler still labels the opcode) but
  *executing* it raises an Instruction Error exception, exactly as the silicon
  does; interrupt and exception return is done with `j.f [ilink1]` /
  `j.f [ilink2]`.
- **Unimplemented opcodes fault.** Every MPY variant, the saturating/DSP group
  (ADDS, SUBS, DIVAW, ASLS, ASRS, ADDSDW, SUBSDW, SAT16, RND16, ABSS, ABSSW,
  NEGS, NEGSW) and EX raise an Instruction Error, as on silicon; EX leaves
  memory untouched. They still decode, so the disassembler labels them. The
  barrel shifts, NORM, NORMW, SWAP, MIN and MAX execute.
- **Exceptions are vectored as measured.** An Instruction Error (including an
  undecodable opcode), SWI and TRAP_S enter slot 2 of the vector table at
  `INT_VECTOR_BASE` (never the NCO alias that serves interrupt vectors), with
  ILINK2 = the address execution would have continued at (the branch target
  in a delay slot), STATUS32_L2 = STATUS32, STATUS32 unchanged and
  `AUX_IRQ_LV12` bit 1 set; `j.f [ilink2]` resumes. What an exception does to
  E1/E2 when interrupts are enabled is not measured. Bus errors, misaligned and
  privileged accesses, whose entry is not measured either, still stop the
  emulation, as does a fault on the vector itself. `arc700 --stop-on-exception`
  (or `Cpu::stop_on_exception`) stops on every exception, for diagnosis.
- **No exception-state auxiliary registers.** ERET, ERBTA, ERSTATUS, ECR, EFA,
  ICAUSE1/2, BTA, BTA_L1/L2 and IENABLE are absent: they read `IDENTITY` and
  ignore writes. Aux `0x11` is MULHI, a plain register reading `0x2` after
  reset; there is no I-cache control register.
- **Flags follow the core, not the textbook.** Register shift counts are taken
  modulo 32, C is the last bit shifted out and is cleared by a zero count, and
  V is kept. The one-bit `ASL.F` and `RLC.F` write V = N xor C. `NORM.F` and
  `NORMW.F` set Z and N from the source operand and keep C and V. `MIN.F` /
  `MAX.F` set C when the second operand is the one selected. `TST`, `CMP`,
  `RCMP` and `BTST` with B = 62 read a LIMM in every format.
- Delay slots, zero-overhead loops and the cache model (direct-mapped I-cache,
  2-way D-cache) follow the observed core behaviour.
- **Not modelled: the `LP_COUNT` setup hazard.** Silicon needs four instruction
  words between the write of `LP_COUNT` and the end of the first iteration; a
  shorter distance can lose an iteration. Whether it does depends on fetch
  timing (it was seen once, on a cold first execution), so the emulator always
  applies the new count. Code that violates the distance behaves here as if it
  did not.

## SoC peripherals — where it simplifies

The peripheral models reproduce the **digital / register** behaviour that
firmware observes, but deliberately simplify the **analog and clock-domain**
behaviour of the SerDes / PCS. Known divergences from real silicon:

- **SerDes MDIO auto-completes.** The block-60 MDIO command registers clear
  their busy bit on the next read instead of stalling until the lane clock and
  PCS lock are actually up. On silicon those transactions gate on the analog
  lane state; here they complete immediately.
- **Lane lock is instantaneous.** Writing the lane-index register marks the lane
  as locked, rather than requiring analog CDR/PCS acquisition. Real lock depends
  on the received signal.
- **Cold calibration does not converge on its own.** The SerDes cold-calibration
  "done" bit is intentionally *not* auto-asserted — it depends on analog
  convergence the emulator cannot model. A scenario flag can force it when you
  need the boot to proceed past calibration.
- **The IND register-file bus completes instantly.** It never asserts "busy";
  on silicon the access takes time and gates on the SerDes clock.
- **eFUSE speed-capability reads are a constant.** The eFUSE snapshot is not
  fully modelled; the default returns a fixed capability value.

In short: anything that depends on the **analog PHY, real clock domains, or
lane-calibration timing** is approximated. The register-level and CPU-level
behaviour is the faithful part.

Contributions improving fidelity are welcome. The guiding rule (see
[`CLAUDE.md`](CLAUDE.md)) is to model observed hardware behaviour, never to
hard-code a specific firmware's needs.
