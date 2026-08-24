#!/usr/bin/env python3
"""Build the Meta Quest red-block operator guide as a dependency-light PDF.

The source stays in Markdown for review and maintenance.  This renderer uses
Matplotlib, which is already part of the Isaac Sim development environment,
and deliberately supports only the small Markdown subset used by the guide.
"""

from __future__ import annotations

import argparse
import re
import textwrap
from dataclasses import dataclass
from datetime import date, datetime
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PROJECT_ROOT / "infodocs" / "meta_quest_redblock_operator_guide.md"
DEFAULT_OUTPUT = PROJECT_ROOT / "infodocs" / "Meta_Quest_RedBlock_Operator_Guide.pdf"

PAGE_SIZE = (8.27, 11.69)  # A4 portrait, inches
LEFT = 0.085
RIGHT = 0.915
TOP = 0.925
BOTTOM = 0.075
TEXT_WIDTH = RIGHT - LEFT

NAVY = "#16324F"
BLUE = "#246A9A"
TEAL = "#16817A"
PALE_BLUE = "#EAF4FA"
PALE_TEAL = "#EAF7F5"
PALE_GRAY = "#F2F4F6"
MID_GRAY = "#637282"
INK = "#17212B"
ORANGE = "#C76B22"


@dataclass
class Block:
    kind: str
    text: str
    level: int = 0


def parse_blocks(markdown: str) -> list[Block]:
    """Parse the intentionally small Markdown subset used by the guide."""

    blocks: list[Block] = []
    paragraph: list[str] = []
    code: list[str] = []
    in_code = False

    def flush_paragraph() -> None:
        if paragraph:
            blocks.append(Block("paragraph", " ".join(line.strip() for line in paragraph)))
            paragraph.clear()

    for raw_line in markdown.splitlines():
        line = raw_line.rstrip()
        if line.startswith("```"):
            flush_paragraph()
            if in_code:
                blocks.append(Block("code", "\n".join(code)))
                code.clear()
            in_code = not in_code
            continue
        if in_code:
            code.append(line)
            continue
        if line == "<!-- PAGEBREAK -->":
            flush_paragraph()
            blocks.append(Block("pagebreak", ""))
            continue
        if line == "<!-- ARCHITECTURE_DIAGRAM -->":
            flush_paragraph()
            blocks.append(Block("architecture", ""))
            continue
        if not line.strip():
            flush_paragraph()
            continue
        heading = re.match(r"^(#{1,4})\s+(.*)$", line)
        if heading:
            flush_paragraph()
            blocks.append(Block("heading", heading.group(2), len(heading.group(1))))
            continue
        bullet = re.match(r"^\s*[-*]\s+(.*)$", line)
        if bullet:
            flush_paragraph()
            blocks.append(Block("bullet", bullet.group(1)))
            continue
        numbered = re.match(r"^\s*(\d+)\.\s+(.*)$", line)
        if numbered:
            flush_paragraph()
            blocks.append(Block("number", numbered.group(2), int(numbered.group(1))))
            continue
        if line.startswith("> "):
            flush_paragraph()
            blocks.append(Block("note", line[2:]))
            continue
        paragraph.append(line)

    flush_paragraph()
    if in_code:
        raise ValueError("Unclosed Markdown code fence")
    return blocks


class GuideRenderer:
    def __init__(self, pdf: PdfPages, title: str) -> None:
        self.pdf = pdf
        self.title = title
        self.fig = None
        self.ax = None
        self.y = TOP
        self.page_number = 0

    def new_page(self, section: str = "Operator guide") -> None:
        if self.fig is not None:
            self.finish_page()
        self.fig, self.ax = plt.subplots(figsize=PAGE_SIZE)
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        self.ax.axis("off")
        self.page_number += 1
        self.y = TOP
        self.ax.add_patch(Rectangle((0, 0.975), 1, 0.025, color=NAVY, linewidth=0))
        self.ax.text(LEFT, 0.952, section, fontsize=7.8, color=MID_GRAY, va="center")

    def finish_page(self) -> None:
        assert self.fig is not None and self.ax is not None
        self.ax.plot([LEFT, RIGHT], [0.052, 0.052], color="#D5DCE2", linewidth=0.7)
        self.ax.text(LEFT, 0.029, self.title, fontsize=6.8, color=MID_GRAY, va="center")
        self.ax.text(
            RIGHT,
            0.029,
            f"{self.page_number}",
            fontsize=7.2,
            color=MID_GRAY,
            ha="right",
            va="center",
        )
        self.pdf.savefig(self.fig, bbox_inches=None)
        plt.close(self.fig)
        self.fig = None
        self.ax = None

    def ensure(self, height: float) -> None:
        if self.fig is None:
            self.new_page()
        if self.y - height < BOTTOM:
            self.new_page()

    @staticmethod
    def clean_inline(text: str) -> str:
        text = text.replace("**", "").replace("__", "")
        return text.replace("`", "")

    def wrapped_lines(self, text: str, width: int, *, subsequent: str = "") -> list[str]:
        return textwrap.wrap(
            self.clean_inline(text),
            width=width,
            break_long_words=False,
            break_on_hyphens=False,
            subsequent_indent=subsequent,
        ) or [""]

    def heading(self, text: str, level: int) -> None:
        if level == 1:
            self.title_page(text)
            return
        sizes = {2: 16.5, 3: 12.0, 4: 9.7}
        spacing = {2: 0.055, 3: 0.041, 4: 0.034}
        before = {2: 0.025, 3: 0.017, 4: 0.011}
        height = spacing.get(level, 0.04)
        self.ensure(height + before.get(level, 0.01))
        self.y -= before.get(level, 0.01)
        assert self.ax is not None
        self.ax.text(
            LEFT,
            self.y,
            self.clean_inline(text),
            fontsize=sizes.get(level, 10),
            fontweight="bold",
            color=NAVY if level == 2 else BLUE,
            va="top",
        )
        if level == 2:
            self.ax.plot([LEFT, RIGHT], [self.y - 0.025, self.y - 0.025], color="#C9DCE8", lw=0.9)
        self.y -= height

    def title_page(self, text: str) -> None:
        if self.fig is not None:
            self.finish_page()
        self.new_page("Cognitive Software Labs · Core Unitree Sim IsaacLab")
        assert self.ax is not None
        self.ax.add_patch(
            FancyBboxPatch(
                (LEFT, 0.60),
                TEXT_WIDTH,
                0.235,
                boxstyle="round,pad=0.012,rounding_size=0.012",
                facecolor=NAVY,
                edgecolor=NAVY,
            )
        )
        title_lines = textwrap.wrap(text, width=27, break_long_words=False)
        self.ax.text(
            LEFT + 0.045,
            0.785,
            "\n".join(title_lines),
            fontsize=25,
            fontweight="bold",
            color="white",
            va="top",
            linespacing=1.12,
        )
        self.ax.text(
            LEFT + 0.047,
            0.625,
            "Installation · launch profiles · runtime settings · architecture · operations",
            fontsize=9.5,
            color="#D9EDF7",
            va="bottom",
        )
        self.ax.text(LEFT, 0.515, "Verified simulator workflow", fontsize=13, color=TEAL, fontweight="bold")
        self.ax.text(
            LEFT,
            0.472,
            "Seven red-block tasks · Meta Quest via Unitree xr_teleoperate · DDS control · ZMQ video",
            fontsize=10,
            color=INK,
        )
        self.ax.text(LEFT, 0.395, "Document date", fontsize=8, color=MID_GRAY)
        self.ax.text(LEFT, 0.367, date(2026, 8, 21).isoformat(), fontsize=10.5, color=INK)
        self.ax.text(0.40, 0.395, "Repository branch", fontsize=8, color=MID_GRAY)
        self.ax.text(
            0.40,
            0.367,
            "fix/teleop-randomized-hospital-integration",
            fontsize=9.2,
            color=INK,
        )
        self.ax.text(
            LEFT,
            0.235,
            "Important: isolate the simulator DDS domain from physical robots unless\n"
            "commanding them is intentional.",
            fontsize=9,
            color=ORANGE,
            fontweight="bold",
            linespacing=1.25,
        )
        self.y = 0.14

    def paragraph(self, text: str) -> None:
        lines = self.wrapped_lines(text, 100)
        height = len(lines) * 0.0162 + 0.008
        self.ensure(height)
        assert self.ax is not None
        self.ax.text(
            LEFT,
            self.y,
            "\n".join(lines),
            fontsize=8.85,
            color=INK,
            va="top",
            linespacing=1.34,
        )
        self.y -= height

    def list_item(self, text: str, marker: str) -> None:
        lines = self.wrapped_lines(text, 92)
        height = len(lines) * 0.0160 + 0.005
        self.ensure(height)
        assert self.ax is not None
        self.ax.text(LEFT + 0.006, self.y, marker, fontsize=8.8, color=TEAL, fontweight="bold", va="top")
        self.ax.text(
            LEFT + 0.030,
            self.y,
            "\n".join(lines),
            fontsize=8.65,
            color=INK,
            va="top",
            linespacing=1.33,
        )
        self.y -= height

    def note(self, text: str) -> None:
        lines = self.wrapped_lines(text, 91)
        height = len(lines) * 0.0162 + 0.022
        self.ensure(height)
        assert self.ax is not None
        bottom = self.y - height + 0.006
        self.ax.add_patch(
            FancyBboxPatch(
                (LEFT, bottom),
                TEXT_WIDTH,
                height - 0.003,
                boxstyle="round,pad=0.006,rounding_size=0.006",
                facecolor=PALE_TEAL,
                edgecolor="#A9D8D1",
                linewidth=0.8,
            )
        )
        self.ax.text(
            LEFT + 0.016,
            self.y - 0.009,
            "\n".join(lines),
            fontsize=8.55,
            color="#155E59",
            va="top",
            linespacing=1.32,
        )
        self.y -= height + 0.008

    def code(self, code: str) -> None:
        raw_lines = code.splitlines() or [""]
        lines: list[str] = []
        for raw in raw_lines:
            lines.extend(textwrap.wrap(raw, width=101, subsequent_indent="    ", break_long_words=False) or [""])
        line_height = 0.0135
        height = len(lines) * line_height + 0.030
        if height > 0.72:
            midpoint = len(raw_lines) // 2
            self.code("\n".join(raw_lines[:midpoint]))
            self.code("\n".join(raw_lines[midpoint:]))
            return
        self.ensure(height)
        assert self.ax is not None
        bottom = self.y - height + 0.005
        self.ax.add_patch(
            FancyBboxPatch(
                (LEFT, bottom),
                TEXT_WIDTH,
                height,
                boxstyle="round,pad=0.007,rounding_size=0.004",
                facecolor=PALE_GRAY,
                edgecolor="#D7DDE2",
                linewidth=0.7,
            )
        )
        self.ax.text(
            LEFT + 0.014,
            self.y - 0.010,
            "\n".join(lines),
            fontsize=7.15,
            family="DejaVu Sans Mono",
            color="#263746",
            va="top",
            linespacing=1.25,
        )
        self.y -= height + 0.010

    def architecture(self) -> None:
        if self.fig is not None and self.y < 0.86:
            self.new_page("Architecture")
        elif self.fig is None:
            self.new_page("Architecture")
        assert self.ax is not None
        self.ax.text(LEFT, self.y, "End-to-end architecture", fontsize=17, fontweight="bold", color=NAVY, va="top")
        self.y -= 0.060
        self.ax.text(
            LEFT,
            self.y,
            "Control travels from the headset to Isaac Sim; rendered camera frames return by a separate path.",
            fontsize=9.3,
            color=INK,
            va="top",
        )

        def box(x: float, y: float, w: float, h: float, title: str, body: str, color: str) -> None:
            self.ax.add_patch(
                FancyBboxPatch(
                    (x, y), w, h,
                    boxstyle="round,pad=0.008,rounding_size=0.008",
                    facecolor="white", edgecolor=color, linewidth=1.5,
                )
            )
            self.ax.add_patch(Rectangle((x, y + h - 0.033), w, 0.033, color=color, linewidth=0))
            self.ax.text(x + 0.010, y + h - 0.016, title, fontsize=7.45, color="white", fontweight="bold", va="center")
            self.ax.text(x + 0.010, y + h - 0.049, body, fontsize=6.7, color=INK, va="top", linespacing=1.25)

        def arrow(x1: float, y1: float, x2: float, y2: float, label: str, color: str) -> None:
            self.ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>", mutation_scale=12, lw=1.5, color=color))
            self.ax.text((x1 + x2) / 2, y1 + 0.017, label, fontsize=6.2, color=color, ha="center", va="bottom")

        xs = [0.055, 0.245, 0.435, 0.625, 0.815]
        widths = [0.13, 0.14, 0.14, 0.14, 0.13]
        y_control = 0.615
        h = 0.125
        box(xs[0], y_control, widths[0], h, "META QUEST", "Hand/controller\ntracking + display", TEAL)
        box(xs[1], y_control, widths[1], h, "VUER / XR", "HTTPS/WSS :8012\ntracking receiver", TEAL)
        box(xs[2], y_control, widths[2], h, "XR CONTROL", "IK + retargeting\nUnitree commands", TEAL)
        box(xs[3], y_control, widths[3], h, "DDS BRIDGE", "domain 1 topics\nshared-memory IPC", TEAL)
        box(xs[4], y_control, widths[4], h, "ISAAC LAB", "ActionProvider +\nrobot controller", TEAL)
        for idx, label in enumerate(("HTTPS/WSS", "poses", "DDS", "actions")):
            arrow(xs[idx] + widths[idx], y_control + h / 2, xs[idx + 1], y_control + h / 2, label, TEAL)

        self.ax.text(LEFT, 0.775, "CONTROL PATH  ·  Quest → simulator", fontsize=8.2, color=TEAL, fontweight="bold")
        self.ax.text(LEFT, 0.505, "VIDEO PATH  ·  simulator → Quest", fontsize=8.2, color=BLUE, fontweight="bold")

        y_video = 0.335
        box(xs[0], y_video, widths[0], h, "META QUEST", "Immersive / ego /\npass-through view", BLUE)
        box(xs[1], y_video, widths[1], h, "VUER / XR", "ImageClient +\nheadset renderer", BLUE)
        box(xs[2], y_video, widths[2], h, "TELEIMAGER", "ZMQ config :60000\nframes :55555-57", BLUE)
        box(xs[3], y_video, widths[3], h, "CAMERA SHM", "multi-image shared\nmemory buffer", BLUE)
        box(xs[4], y_video, widths[4], h, "RTX CAMERAS", "head + left/right\nwrist at 480x640", BLUE)
        for idx, label in enumerate(("render", "ZMQ", "SHM", "RGB")):
            arrow(xs[idx + 1], y_video + h / 2, xs[idx] + widths[idx], y_video + h / 2, label, BLUE)

        self.ax.add_patch(
            FancyBboxPatch(
                (LEFT, 0.185), TEXT_WIDTH, 0.080,
                boxstyle="round,pad=0.008,rounding_size=0.007",
                facecolor=PALE_BLUE, edgecolor="#B7D3E4", linewidth=0.8,
            )
        )
        self.ax.text(
            LEFT + 0.015,
            0.245,
            "Why there are two servers",
            fontsize=8.5,
            color=NAVY,
            fontweight="bold",
            va="top",
        )
        self.ax.text(
            LEFT + 0.015,
            0.218,
            "The simulator exposes camera frames through teleimager/ZMQ. xr_teleoperate consumes those frames and hosts the",
            fontsize=7.8,
            color=INK,
            va="top",
        )
        self.ax.text(
            LEFT + 0.015,
            0.196,
            "browser-facing Vuer session. --meta_quest disables teleimager direct WebRTC, but Vuer still needs its own TLS certificate.",
            fontsize=7.8,
            color=INK,
            va="top",
        )
        self.y = 0.145

    def render(self, blocks: list[Block]) -> None:
        for block in blocks:
            if block.kind == "pagebreak":
                self.new_page()
            elif block.kind == "architecture":
                self.architecture()
            elif block.kind == "heading":
                self.heading(block.text, block.level)
            elif block.kind == "paragraph":
                self.paragraph(block.text)
            elif block.kind == "bullet":
                self.list_item(block.text, "•")
            elif block.kind == "number":
                self.list_item(block.text, f"{block.level}.")
            elif block.kind == "note":
                self.note(block.text)
            elif block.kind == "code":
                self.code(block.text)
            else:
                raise ValueError(f"Unsupported block kind: {block.kind}")
        if self.fig is not None:
            self.finish_page()


def build_pdf(source: Path, output: Path) -> None:
    markdown = source.read_text(encoding="utf-8")
    blocks = parse_blocks(markdown)
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata = {
        "Title": "Meta Quest Red-Block Teleoperation Operator Guide",
        "Author": "Cognitive Software Labs",
        "Subject": "Running Core Unitree Sim IsaacLab red-block tasks with Meta Quest and xr_teleoperate",
        "Keywords": "Meta Quest, Isaac Sim, Isaac Lab, Unitree, xr_teleoperate, DDS, teleimager",
        "CreationDate": datetime(2026, 8, 21),
    }
    with PdfPages(output, metadata=metadata) as pdf:
        renderer = GuideRenderer(pdf, "Meta Quest red-block teleoperation")
        renderer.render(blocks)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    build_pdf(args.source.resolve(), args.output.resolve())
    print(args.output.resolve())


if __name__ == "__main__":
    main()
