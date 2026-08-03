//! The peer as a program.
//!
//! Three things it is for:
//!
//! - `run` — watch what the peer does on its own, at whatever speed you ask
//!   for. Nothing answers it, which is the point: this is what an ONU that has
//!   just been plugged in sees.
//! - `check` — run the peer against a minimal responder and report what the
//!   far end measured. The registration lifetime is the interesting one: the
//!   peer's timer is a round minute, and the difference is what the fibre and
//!   the exchange cost. Neither figure is written anywhere; both are measured.
//! - `dissect` — read a frame off the command line.

use std::process::ExitCode;
use std::time::{Duration, Instant};

use epon_olt::clock::{WireDuration, WireInstant, PS_PER_MS};
use epon_olt::link::Link;
use epon_olt::onu::OnuResponder;
use epon_olt::types::MacAddr;
use epon_olt::{decode, FibreConfig, PeerConfig};

/// Wire time one step of the loop covers.
const STEP: WireDuration = WireDuration::from_us(100);

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut args = args.iter().map(String::as_str);
    match args.next() {
        Some("run") => run(Args::parse(args)),
        Some("check") => return check(Args::parse(args)),
        Some("dissect") => return dissect(args.collect()),
        Some("--help" | "-h") | None => usage(),
        Some(other) => {
            eprintln!("unknown command: {other}\n");
            usage();
            return ExitCode::FAILURE;
        }
    }
    ExitCode::SUCCESS
}

fn usage() {
    println!(
        "\
epon-olt — an EPON OLT peer that runs its own loop

  olt run   [--seconds N] [--realtime] [--quiet]
            Run the peer with nothing answering it.

  olt check [--seconds N] [--realtime]
            Run it against a minimal responder and report what the far end
            measured. Exits non-zero if the link never registered.

  olt dissect <hex bytes>
            Dissect one frame.

Options
  --seconds N   link time to cover (default 70, past one registration life)
  --realtime    pace the run against a wall clock instead of running flat out
  --quiet       counters only, no per-frame lines
  --distance N  fibre length in km, both ways (default 20)
  --jitter N    upper bound of the per-frame jitter, in microseconds
  --depth N     frames a direction holds before it drops"
    );
}

struct Args {
    seconds: u64,
    realtime: bool,
    quiet: bool,
    distance_km: u64,
    jitter_us: u64,
    depth: usize,
}

impl Args {
    fn parse<'a>(args: impl Iterator<Item = &'a str>) -> Self {
        let mut out = Self {
            seconds: 70,
            realtime: false,
            quiet: false,
            distance_km: epon_olt::fibre::DEFAULT_DISTANCE_KM,
            jitter_us: 0,
            depth: epon_olt::fibre::DEFAULT_DEPTH,
        };
        let mut args = args.peekable();
        while let Some(arg) = args.next() {
            let mut value = |name: &str| -> u64 {
                args.next()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or_else(|| panic!("{name} wants a number"))
            };
            match arg {
                "--seconds" => out.seconds = value("--seconds"),
                "--realtime" => out.realtime = true,
                "--quiet" => out.quiet = true,
                "--distance" => out.distance_km = value("--distance"),
                "--jitter" => out.jitter_us = value("--jitter"),
                "--depth" => out.depth = value("--depth") as usize,
                other => eprintln!("ignoring unknown option {other}"),
            }
        }
        out
    }

    fn link(&self) -> Link {
        let shape = |base: FibreConfig| FibreConfig {
            propagation: FibreConfig::propagation_for_km(self.distance_km),
            jitter: WireDuration::from_us(self.jitter_us),
            depth: self.depth,
            ..base
        };
        Link::new(
            PeerConfig::default(),
            shape(FibreConfig::downstream()),
            shape(FibreConfig::upstream()),
        )
    }

    fn end(&self) -> WireInstant {
        WireInstant::from_ps(WireDuration::from_ms(self.seconds * 1000).as_ps())
    }
}

/// Pace a virtual clock against a wall clock, when asked to.
struct Pacer {
    started: Instant,
    realtime: bool,
}

impl Pacer {
    fn new(realtime: bool) -> Self {
        Self { started: Instant::now(), realtime }
    }

    /// Wait until wall time has caught up with `now`.
    fn wait_for(&self, now: WireInstant) {
        if !self.realtime {
            return;
        }
        let target = Duration::from_nanos(now.as_ps() / 1000);
        let elapsed = self.started.elapsed();
        if target > elapsed {
            std::thread::sleep(target - elapsed);
        }
    }
}

fn run(args: Args) {
    let mut link = args.link();
    let pacer = Pacer::new(args.realtime);
    link.set_link(true, WireInstant::ZERO);

    let mut now = WireInstant::ZERO;
    while now <= args.end() {
        pacer.wait_for(now);
        link.advance_to(now);
        for (at, line) in link.peer.take_log_lines() {
            println!("{at} [OLT] {line}");
        }
        // Nothing is answering, so drain the downstream ourselves — otherwise
        // the queue fills and every frame past its depth is dropped, which is
        // a property of the far end, not of the peer.
        while let Some(landed) = link.poll_downstream(now) {
            if !args.quiet {
                let d = decode::dissect(&landed.frame);
                println!("{} -> {}", landed.arrives_at, d.summary);
            }
        }
        now += STEP;
    }
    report(&link, None);
}

fn check(args: Args) -> ExitCode {
    let mut link = args.link();
    let mut onu = OnuResponder::new(MacAddr::new([0x02, 0x00, 0x00, 0x01, 0x02, 0x03]));
    let pacer = Pacer::new(args.realtime);
    link.set_link(true, WireInstant::ZERO);

    let mut now = WireInstant::ZERO;
    while now <= args.end() {
        pacer.wait_for(now);
        link.advance_to(now);
        for (at, line) in link.peer.take_log_lines() {
            println!("{at} [OLT] {line}");
        }
        while let Some(landed) = link.poll_downstream(now) {
            onu.deliver(&landed.frame, landed.arrives_at);
        }
        onu.advance_to(now);
        for (at, frame) in onu.take_output() {
            link.send_upstream(frame, at.max(now));
        }
        now += STEP;
    }

    report(&link, Some(&onu));
    if onu.counters.acks_sent == 0 {
        eprintln!("\nFAIL: the link never registered");
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}

fn report(link: &Link, onu: Option<&OnuResponder>) {
    let c = &link.peer.counters;
    println!("\npeer");
    println!("  state                {}", link.peer.mpcp_state());
    println!("  frames sent          {}", c.frames_sent);
    println!("  frames received      {}", c.frames_received);
    println!("  GATEs                {}", c.gates_sent);
    println!("  OAM keepalives       {}", c.oam_keepalives_sent);
    println!("  attribute requests   {}", c.attribute_requests_sent);
    println!("  attribute replies    {}", c.attribute_replies_seen);
    println!("  registrations        {}", c.registrations);
    println!("  deregistrations      {}", c.deregistrations);
    println!("  REGISTER_REQ seen    {}", c.register_req_seen);
    println!("    accepted           {}", c.register_req_accepted);
    println!("    window passed over {}", c.register_req_window_passed);
    println!("    handshake in fight {}", c.register_req_in_flight);
    println!("    link settling      {}", c.register_req_link_settling);
    println!("    malformed          {}", c.register_req_malformed);
    println!(
        "  every request accounted for: {}",
        if c.register_requests_accounted_for() { "yes" } else { "NO" }
    );
    println!("  upstream MPCPDUs     {}", c.mpcp_upstream_seen);
    println!("    REPORT             {}", c.reports_seen);
    println!("    wrong direction    {}", c.mpcp_upstream_wrong_direction);
    println!("    unhandled opcode   {}", c.mpcp_upstream_unhandled);
    println!("    malformed          {}", c.mpcp_malformed);
    println!(
        "  every upstream MPCPDU accounted for: {}",
        if c.upstream_mpcpdus_accounted_for() { "yes" } else { "NO" }
    );
    println!("  GATEs sent           {}", c.gates_sent);
    println!("    non-discovery      {}", c.gates_normal_sent);

    println!("\nfibre");
    for (name, f) in [("downstream", &link.downstream), ("upstream", &link.upstream)] {
        println!(
            "  {name:<11} sent {:>6}  delivered {:>6}  dropped(full) {:>4}  in flight {}",
            f.sent, f.delivered, f.dropped_full, f.in_flight()
        );
    }

    let Some(onu) = onu else { return };
    println!("\nfar end");
    println!("  GATEs seen           {}", onu.counters.gates_seen);
    println!("  REGISTERs seen       {}", onu.counters.registers_seen);
    println!("  teardowns seen       {}", onu.counters.deregisters_seen);
    println!("  OAMPDUs seen         {}", onu.counters.oam_seen);
    println!("  extended OAMPDUs     {}", onu.counters.extended_seen);

    let lives = &onu.registration_lives;
    println!("\nregistration lifetime, as the far end measured it");
    if lives.is_empty() {
        println!("  (nothing was torn down within the run)");
        return;
    }
    let configured = link.peer.config.registration_lifetime;
    println!("  the peer's own timer {configured}");
    for (i, life) in lives.iter().enumerate() {
        println!(
            "  registration {i:<2}      {life}   (+{:.1} µs)",
            life.saturating_sub(configured).as_ps() as f64 / 1e6
        );
    }
    let worst = lives.iter().map(|l| l.saturating_sub(configured)).max().unwrap_or_default();
    println!(
        "  {}",
        if worst.as_ps() < PS_PER_MS {
            "the exchange costs well under a millisecond: a longer gap measured on a real \
             link is not travel time"
        } else {
            "the exchange itself costs a measurable amount here"
        }
    );
}

fn dissect(args: Vec<&str>) -> ExitCode {
    let hex: String = args.join("").chars().filter(|c| !c.is_whitespace()).collect();
    if hex.is_empty() || hex.len() % 2 != 0 {
        eprintln!("give an even number of hex digits");
        return ExitCode::FAILURE;
    }
    let mut frame = Vec::with_capacity(hex.len() / 2);
    for i in (0..hex.len()).step_by(2) {
        match u8::from_str_radix(&hex[i..i + 2], 16) {
            Ok(b) => frame.push(b),
            Err(_) => {
                eprintln!("not hex at offset {i}: {}", &hex[i..i + 2]);
                return ExitCode::FAILURE;
            }
        }
    }
    let d = decode::dissect(&frame);
    println!("{} — {}", d.protocol, d.summary);
    for field in &d.fields {
        let indent = "  ".repeat(field.depth as usize + 1);
        match field.offset {
            Some(off) => println!("{indent}{:<28} {}  [{off}..{}]", field.name, field.value, off + field.len),
            None => println!("{indent}{:<28} {}", field.name, field.value),
        }
    }
    ExitCode::SUCCESS
}
