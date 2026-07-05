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
- Delay slots, zero-overhead loops, the cache model (direct-mapped I-cache,
  2-way D-cache) and the NORM extension follow the observed core behaviour.

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
