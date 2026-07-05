# Provenance

This is an independent, clean-room emulator.

I have never had the manufacturer's datasheet or any confidential documentation
for this SoC, and I am not bound by any non-disclosure agreement covering it.

Everything the emulator knows about the chip — register layouts, peripheral
behaviour, reset values, instruction semantics — I worked out by reverse
engineering hardware I own: watching how the firmware that shipped on the device
drives the chip over its debug UART, disassembling that firmware to follow its
hardware accesses, and running my own firmware on the same hardware to probe
registers and see how they respond.

What the emulator reproduces is the *hardware's* behaviour, not the firmware. It
contains no firmware — mine or the vendor's — and no firmware code is reproduced
or published; the disassembly was only a means to understand how the chip works.

The chip's on-device identity (serial numbers, MAC addresses, SFP EEPROM,
eFUSE) is not reproduced: the values shipped here are synthetic placeholders,
and real ones can be supplied at runtime if needed.

Protocol details come only from public standards — IEEE 802.3 (EPON / MPCP),
SFF-8472 / 8024 / 8079 (SFP), and the public ARCompact / ARC700 instruction set.

Reverse-engineering a product you lawfully own, for study and interoperability,
is permitted under EU law (Trade Secrets Directive 2016/943 Art. 3 and the
software-interoperability exceptions of Directive 2009/24/EC). This project is
not affiliated with or endorsed by the manufacturer; product names are used only
to identify the hardware being modelled.
