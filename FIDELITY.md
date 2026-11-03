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
- **The statistics counters behind the port block are plain storage.** The busy
  bit firmware spins on before taking a count — bit 31 of `0x01002D1C` — *is*
  modelled as the hardware's, so the wait itself behaves as it does on silicon.
  The counter words behind it are not: they hold their seeded values and never
  advance with traffic, so a statistics read returns zero. It also reads zero on
  silicon, where those counters were measured flat while the port was carrying
  signal. **Agreement on that number therefore proves nothing**, and the
  statistics commands are not an oracle for conformance in either direction.
- **One word of the port block has no rule.** `0x01002D0C` reads `0x00008807` on
  silicon once the port is up, from a pre-arm value of `0x00000003`, and no write
  produces it — so it is hardware-driven, but what drives it is not established.
  It is left as plain storage rather than given a guessed rule, and stays at the
  seeded value here.

In short: anything that depends on the **analog PHY, real clock domains, or
lane-calibration timing** is approximated. The register-level and CPU-level
behaviour is the faithful part.

## Two register behaviours chosen to keep a firmware moving

Everything above simplifies something measured. These two do not: each picks a
behaviour because letting the model do the obvious thing sent a firmware down a
stop path, and in each case what the silicon does is open. They are uniform
rules — no address, image or symbol triggers them — but the reason they exist is
an observation of software, so they are listed apart.

- **`0x01002804` reads zero and swallows writes** (`src/soc/serdes.rs`). Letting
  a write come back out of the same word made a hardware-state poller see bits
  appear and take a shutdown path. Keeping the two directions apart is what
  stops that; which direction the hardware implements is not established, and
  the two names this word once carried have been retracted.
- **`0x01003604` reads zero until something injects** (`src/soc/fatal_filter.rs`).
  The power-on table holds a value for this word and the model deliberately does
  **not** seed it: under either reading of the word that value is a status, and a
  seeded status makes a firmware act on a condition nothing produced. This keeps
  a polling loop moving; it is not a measurement of the reset value.

Both are falsifiable the same way: a silicon reading of either word taken in a
known state. Until then, neither is the silicon's behaviour.

## Not a divergence — two state-matching traps

**The UNI-side port starts with no link partner.** The words at `0x01002D14` and
`0x01002D2C` follow what arrives on the receive pair — one tracks the presence of
clause 37 auto-negotiation and takes a distinct signature under 10G line coding,
the other carries that state plus the port's own armed bit. That is modelled, but
the signal is an **input**, not something the model invents: with nothing injected
the port reads unarmed with no partner, which is a faithful reading of that state
and not a missing feature. Supply one with `--uni-rx none|basex|basex-an|10g`
before comparing these words against a capture taken with a partner attached.

The seeds for `0x01002D00..0x01002D40` are a post-boot, pre-arm silicon reading,
not a power-on scan — that block is outside the two ranges that were
brute-scanned.

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
