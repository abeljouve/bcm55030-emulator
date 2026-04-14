//! Visual theme: Catppuccin palette (Mocha / Frappe / Macchiato /
//! Latte) plus the emulator-specific accent tokens used across
//! panels. The Catppuccin hex values are vendored directly — the
//! upstream `catppuccin-egui` crate tops out at egui 0.33, and we
//! are pinned to egui 0.34.
//!
//! <https://github.com/catppuccin/catppuccin>

use egui::{Color32, CornerRadius, Stroke, Style, Visuals};

/// One of the four Catppuccin flavours. Persisted across sessions
/// via eframe's built-in storage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Palette {
    Mocha,
    Frappe,
    Macchiato,
    Latte,
}

impl Palette {
    pub const ALL: [Palette; 4] = [
        Palette::Mocha,
        Palette::Frappe,
        Palette::Macchiato,
        Palette::Latte,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Palette::Mocha => "Mocha",
            Palette::Frappe => "Frappé",
            Palette::Macchiato => "Macchiato",
            Palette::Latte => "Latte",
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Palette::Mocha => "mocha",
            Palette::Frappe => "frappe",
            Palette::Macchiato => "macchiato",
            Palette::Latte => "latte",
        }
    }

    pub fn from_str(s: &str) -> Option<Self> {
        match s {
            "mocha" => Some(Palette::Mocha),
            "frappe" => Some(Palette::Frappe),
            "macchiato" => Some(Palette::Macchiato),
            "latte" => Some(Palette::Latte),
            _ => None,
        }
    }

    pub fn is_dark(self) -> bool {
        !matches!(self, Palette::Latte)
    }

    pub fn colors(self) -> Catppuccin {
        match self {
            Palette::Mocha => MOCHA,
            Palette::Frappe => FRAPPE,
            Palette::Macchiato => MACCHIATO,
            Palette::Latte => LATTE,
        }
    }
}

/// 26-colour Catppuccin flavour. Field order follows the upstream
/// specification.
#[derive(Clone, Copy, Debug)]
pub struct Catppuccin {
    pub rosewater: Color32,
    pub flamingo: Color32,
    pub pink: Color32,
    pub mauve: Color32,
    pub red: Color32,
    pub maroon: Color32,
    pub peach: Color32,
    pub yellow: Color32,
    pub green: Color32,
    pub teal: Color32,
    pub sky: Color32,
    pub sapphire: Color32,
    pub blue: Color32,
    pub lavender: Color32,
    pub text: Color32,
    pub subtext1: Color32,
    pub subtext0: Color32,
    pub overlay2: Color32,
    pub overlay1: Color32,
    pub overlay0: Color32,
    pub surface2: Color32,
    pub surface1: Color32,
    pub surface0: Color32,
    pub base: Color32,
    pub mantle: Color32,
    pub crust: Color32,
}

const fn rgb(r: u8, g: u8, b: u8) -> Color32 {
    Color32::from_rgb(r, g, b)
}

pub const MOCHA: Catppuccin = Catppuccin {
    rosewater: rgb(0xf5, 0xe0, 0xdc),
    flamingo: rgb(0xf2, 0xcd, 0xcd),
    pink: rgb(0xf5, 0xc2, 0xe7),
    mauve: rgb(0xcb, 0xa6, 0xf7),
    red: rgb(0xf3, 0x8b, 0xa8),
    maroon: rgb(0xeb, 0xa0, 0xac),
    peach: rgb(0xfa, 0xb3, 0x87),
    yellow: rgb(0xf9, 0xe2, 0xaf),
    green: rgb(0xa6, 0xe3, 0xa1),
    teal: rgb(0x94, 0xe2, 0xd5),
    sky: rgb(0x89, 0xdc, 0xeb),
    sapphire: rgb(0x74, 0xc7, 0xec),
    blue: rgb(0x89, 0xb4, 0xfa),
    lavender: rgb(0xb4, 0xbe, 0xfe),
    text: rgb(0xcd, 0xd6, 0xf4),
    subtext1: rgb(0xba, 0xc2, 0xde),
    subtext0: rgb(0xa6, 0xad, 0xc8),
    overlay2: rgb(0x93, 0x99, 0xb2),
    overlay1: rgb(0x7f, 0x84, 0x9c),
    overlay0: rgb(0x6c, 0x70, 0x86),
    surface2: rgb(0x58, 0x5b, 0x70),
    surface1: rgb(0x45, 0x47, 0x5a),
    surface0: rgb(0x31, 0x32, 0x44),
    base: rgb(0x1e, 0x1e, 0x2e),
    mantle: rgb(0x18, 0x18, 0x25),
    crust: rgb(0x11, 0x11, 0x1b),
};

pub const FRAPPE: Catppuccin = Catppuccin {
    rosewater: rgb(0xf2, 0xd5, 0xcf),
    flamingo: rgb(0xee, 0xbe, 0xbe),
    pink: rgb(0xf4, 0xb8, 0xe4),
    mauve: rgb(0xca, 0x9e, 0xe6),
    red: rgb(0xe7, 0x82, 0x84),
    maroon: rgb(0xea, 0x99, 0x9c),
    peach: rgb(0xef, 0x9f, 0x76),
    yellow: rgb(0xe5, 0xc8, 0x90),
    green: rgb(0xa6, 0xd1, 0x89),
    teal: rgb(0x81, 0xc8, 0xbe),
    sky: rgb(0x99, 0xd1, 0xdb),
    sapphire: rgb(0x85, 0xc1, 0xdc),
    blue: rgb(0x8c, 0xaa, 0xee),
    lavender: rgb(0xba, 0xbb, 0xf1),
    text: rgb(0xc6, 0xd0, 0xf5),
    subtext1: rgb(0xb5, 0xbf, 0xe2),
    subtext0: rgb(0xa5, 0xad, 0xce),
    overlay2: rgb(0x94, 0x9c, 0xbb),
    overlay1: rgb(0x83, 0x8b, 0xa7),
    overlay0: rgb(0x73, 0x7a, 0x94),
    surface2: rgb(0x62, 0x68, 0x80),
    surface1: rgb(0x51, 0x57, 0x6d),
    surface0: rgb(0x41, 0x45, 0x59),
    base: rgb(0x30, 0x34, 0x46),
    mantle: rgb(0x29, 0x2c, 0x3c),
    crust: rgb(0x23, 0x26, 0x34),
};

pub const MACCHIATO: Catppuccin = Catppuccin {
    rosewater: rgb(0xf4, 0xdb, 0xd6),
    flamingo: rgb(0xf0, 0xc6, 0xc6),
    pink: rgb(0xf5, 0xbd, 0xe6),
    mauve: rgb(0xc6, 0xa0, 0xf6),
    red: rgb(0xed, 0x87, 0x96),
    maroon: rgb(0xee, 0x99, 0xa0),
    peach: rgb(0xf5, 0xa9, 0x7f),
    yellow: rgb(0xee, 0xd4, 0x9f),
    green: rgb(0xa6, 0xda, 0x95),
    teal: rgb(0x8b, 0xd5, 0xca),
    sky: rgb(0x91, 0xd7, 0xe3),
    sapphire: rgb(0x7d, 0xc4, 0xe4),
    blue: rgb(0x8a, 0xad, 0xf4),
    lavender: rgb(0xb7, 0xbd, 0xf8),
    text: rgb(0xca, 0xd3, 0xf5),
    subtext1: rgb(0xb8, 0xc0, 0xe0),
    subtext0: rgb(0xa5, 0xad, 0xcb),
    overlay2: rgb(0x93, 0x9a, 0xb7),
    overlay1: rgb(0x80, 0x87, 0xa2),
    overlay0: rgb(0x6e, 0x73, 0x8d),
    surface2: rgb(0x5b, 0x60, 0x78),
    surface1: rgb(0x49, 0x4d, 0x64),
    surface0: rgb(0x36, 0x3a, 0x4f),
    base: rgb(0x24, 0x27, 0x3a),
    mantle: rgb(0x1e, 0x20, 0x30),
    crust: rgb(0x18, 0x19, 0x26),
};

pub const LATTE: Catppuccin = Catppuccin {
    rosewater: rgb(0xdc, 0x8a, 0x78),
    flamingo: rgb(0xdd, 0x78, 0x78),
    pink: rgb(0xea, 0x76, 0xcb),
    mauve: rgb(0x88, 0x39, 0xef),
    red: rgb(0xd2, 0x0f, 0x39),
    maroon: rgb(0xe6, 0x45, 0x53),
    peach: rgb(0xfe, 0x64, 0x0b),
    yellow: rgb(0xdf, 0x8e, 0x1d),
    green: rgb(0x40, 0xa0, 0x2b),
    teal: rgb(0x17, 0x92, 0x99),
    sky: rgb(0x04, 0xa5, 0xe5),
    sapphire: rgb(0x20, 0x9f, 0xb5),
    blue: rgb(0x1e, 0x66, 0xf5),
    lavender: rgb(0x71, 0x87, 0xf7),
    text: rgb(0x4c, 0x4f, 0x69),
    subtext1: rgb(0x5c, 0x5f, 0x77),
    subtext0: rgb(0x6c, 0x6f, 0x85),
    overlay2: rgb(0x7c, 0x7f, 0x93),
    overlay1: rgb(0x8c, 0x8f, 0xa1),
    overlay0: rgb(0x9c, 0xa0, 0xb0),
    surface2: rgb(0xac, 0xb0, 0xbe),
    surface1: rgb(0xbc, 0xc0, 0xcc),
    surface0: rgb(0xcc, 0xd0, 0xda),
    base: rgb(0xef, 0xf1, 0xf5),
    mantle: rgb(0xe6, 0xe9, 0xef),
    crust: rgb(0xdc, 0xe0, 0xe8),
};

/// Build an egui `Visuals` from a Catppuccin flavour. Sets window
/// fill, panel fill, widget backgrounds, strokes, corner rounding,
/// and selection colours. Applied via `Context::set_visuals`.
pub fn visuals_for(palette: Palette) -> Visuals {
    let c = palette.colors();
    let is_dark = palette.is_dark();
    let mut v = if is_dark { Visuals::dark() } else { Visuals::light() };

    v.override_text_color = Some(c.text);
    v.hyperlink_color = c.sapphire;
    v.faint_bg_color = c.surface0;
    v.extreme_bg_color = c.crust;
    v.code_bg_color = c.mantle;
    v.warn_fg_color = c.peach;
    v.error_fg_color = c.red;
    v.window_fill = c.base;
    v.panel_fill = c.base;
    v.window_stroke = Stroke::new(1.0, c.surface1);
    v.window_corner_radius = CornerRadius::same(8);

    v.widgets.noninteractive.bg_fill = c.base;
    v.widgets.noninteractive.weak_bg_fill = c.mantle;
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, c.surface0);
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, c.text);
    v.widgets.noninteractive.corner_radius = CornerRadius::same(6);

    v.widgets.inactive.bg_fill = c.surface0;
    v.widgets.inactive.weak_bg_fill = c.surface0;
    v.widgets.inactive.bg_stroke = Stroke::new(1.0, c.surface1);
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, c.subtext1);
    v.widgets.inactive.corner_radius = CornerRadius::same(6);

    v.widgets.hovered.bg_fill = c.surface1;
    v.widgets.hovered.weak_bg_fill = c.surface1;
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, c.lavender);
    v.widgets.hovered.fg_stroke = Stroke::new(1.0, c.text);
    v.widgets.hovered.corner_radius = CornerRadius::same(6);

    v.widgets.active.bg_fill = c.surface2;
    v.widgets.active.weak_bg_fill = c.surface2;
    v.widgets.active.bg_stroke = Stroke::new(1.0, c.mauve);
    v.widgets.active.fg_stroke = Stroke::new(1.0, c.text);
    v.widgets.active.corner_radius = CornerRadius::same(6);

    v.widgets.open.bg_fill = c.surface1;
    v.widgets.open.weak_bg_fill = c.surface1;
    v.widgets.open.bg_stroke = Stroke::new(1.0, c.overlay0);
    v.widgets.open.fg_stroke = Stroke::new(1.0, c.text);

    v.selection.bg_fill = c.blue.gamma_multiply(0.35);
    v.selection.stroke = Stroke::new(1.0, c.blue);

    v
}

/// Tune the default text styles: bump the monospace size slightly
/// and let JetBrains Mono shine through where it is loaded.
pub fn configure_style(style: &mut Style) {
    use egui::{FontFamily, FontId, TextStyle};
    style.text_styles.insert(
        TextStyle::Heading,
        FontId::new(18.0, FontFamily::Proportional),
    );
    style.text_styles.insert(
        TextStyle::Body,
        FontId::new(13.0, FontFamily::Proportional),
    );
    style.text_styles.insert(
        TextStyle::Monospace,
        FontId::new(13.0, FontFamily::Monospace),
    );
    style.text_styles.insert(
        TextStyle::Button,
        FontId::new(13.0, FontFamily::Proportional),
    );
    style.text_styles.insert(
        TextStyle::Small,
        FontId::new(11.0, FontFamily::Proportional),
    );
    style.spacing.item_spacing = egui::vec2(6.0, 4.0);
    style.spacing.button_padding = egui::vec2(8.0, 4.0);
    style.visuals.indent_has_left_vline = false;
}

// ---------------------------------------------------------------
// Emulator-specific accent tokens. Derived from the active palette
// at draw time via `AccentTokens::from(palette)`.
// ---------------------------------------------------------------

/// Colours the panels reach for instead of hard-coding. Rebuilt per
/// frame from the active palette so switching themes is instant.
#[derive(Clone, Copy)]
pub struct AccentTokens {
    pub breakpoint: Color32,
    pub pc_highlight: Color32,
    pub pc_highlight_strong: Color32,
    pub delay_slot: Color32,
    pub lp_range: Color32,
    pub changed_reg: Color32,
    pub stack: Color32,
    pub terminal_fg: Color32,
    pub terminal_bg: Color32,
    pub mutation: Color32,
    pub muted: Color32,
    pub accent: Color32,
    pub success: Color32,
    pub warning: Color32,
    pub danger: Color32,
}

impl AccentTokens {
    pub fn from_palette(p: Palette) -> Self {
        let c = p.colors();
        Self {
            breakpoint: c.red,
            pc_highlight: c.yellow.gamma_multiply(0.22),
            pc_highlight_strong: c.yellow,
            delay_slot: c.mauve.gamma_multiply(0.22),
            lp_range: c.green.gamma_multiply(0.16),
            changed_reg: c.peach,
            stack: c.sapphire.gamma_multiply(0.18),
            terminal_fg: c.green,
            terminal_bg: c.crust,
            mutation: c.peach,
            muted: c.overlay1,
            accent: c.mauve,
            success: c.green,
            warning: c.yellow,
            danger: c.red,
        }
    }
}

// Backwards-compatibility constants used by panels that haven't yet
// switched to `AccentTokens`. These resolve to Mocha values so the
// default look is unchanged when code is read in isolation.
pub const BREAKPOINT: Color32 = rgb(0xf3, 0x8b, 0xa8);
pub const PC_HIGHLIGHT: Color32 = rgb(0x40, 0x35, 0x00);
pub const DELAY_SLOT: Color32 = rgb(0x3c, 0x2b, 0x4c);
pub const LP_RANGE: Color32 = rgb(0x20, 0x3a, 0x28);
pub const CHANGED_REG: Color32 = rgb(0xfa, 0xb3, 0x87);
pub const STACK: Color32 = rgb(0x1e, 0x3a, 0x4f);
pub const TERMINAL_FG: Color32 = rgb(0xa6, 0xe3, 0xa1);
pub const TERMINAL_BG: Color32 = rgb(0x11, 0x11, 0x1b);
pub const MUTATION: Color32 = rgb(0xfa, 0xb3, 0x87);
pub const MUTED: Color32 = rgb(0x7f, 0x84, 0x9c);
