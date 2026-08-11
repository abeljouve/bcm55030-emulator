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
- **The statistics counters behind the port block are plain storage.** The busy
  bit firmware spins on before taking a count — bit 31 of `0x01002D1C` — *is*
  modelled as the hardware's, so the wait itself behaves as it does on silicon.
  The counter words behind it are not: they hold their reset values and never
  advance with traffic, so a statistics read returns zero. It also reads zero on
  silicon, where those counters were measured flat while the port was carrying
  signal. **Agreement on that number therefore proves nothing**, and the
  statistics commands are not an oracle for conformance in either direction.
- **One word of the port block has no rule.** `0x01002D0C` reads `0x00008807` on
  silicon once the port is up, from a reset value of `0x00000003`, and no write
  produces it — so it is hardware-driven, but what drives it is not established.
  It is left as plain storage rather than given a guessed rule, and stays at the
  reset value here.

In short: anything that depends on the **analog PHY, real clock domains, or
lane-calibration timing** is approximated. The register-level and CPU-level
behaviour is the faithful part.

## Not a divergence — two state-matching traps

**The UNI-side port starts with no link partner.** The words at `0x01002D14` and
`0x01002D2C` follow what arrives on the receive pair — one tracks the presence of
clause 37 auto-negotiation and takes a distinct signature under 10G line coding,
the other carries that state plus the port's own armed bit. That is modelled, but
the signal is an **input**, not something the model invents: with nothing injected
the port reads unarmed with no partner, which is a faithful reading of that state
and not a missing feature. Supply one with `--uni-rx none|basex|basex-an|10g`
before comparing these words against a capture taken with a partner attached.

The contents of the classifier tables **depend on the state of the link**: one
rule with the peer down, five once the link is established, six in between.
Software installs the sixth on its teardown path and erases it again on the
bring-up path, so the emulator is not missing it — the two inventories were
simply taken in different states.

An inventory of those tables compared without saying which state it was taken in
is therefore a confounded measurement, not a fidelity result. Whatever
produces such an inventory has to record the link state alongside the
table contents; until it does,
record it by hand.

Contributions improving fidelity are welcome. The guiding rule (see
[`CLAUDE.md`](CLAUDE.md)) is to model observed hardware behaviour, never to
hard-code a specific firmware's needs.
