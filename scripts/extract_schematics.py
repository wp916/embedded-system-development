from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageChops


SKILL_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = SKILL_ROOT / "原理图"
OUTPUT_ROOT = SKILL_ROOT / "Template" / "原理图"

BOARD_CONFIG = {
    "103 原理图": {
        "output": "战舰STM32F103",
        "full_projects": {"WARSHIP_DNF103P.pdf": 6},
        "exclude": set(),
    },
    "429 原理图": {
        "output": "阿波罗STM32F429",
        "full_projects": {
            "DNF429P_F767P_H743P V2.6.pdf": 5,
            "STM32F429_CORE_V2.pdf": 5,
        },
        "exclude": {"DNF429P_F767P_H743P V2.5.pdf"},
    },
}

BLOCK_TITLES = {
    ("WARSHIP_DNF103P.pdf", 1): ["PWM_DAC_AUDIO", "SPEAKER", "ETHERNET", "MULTIMEDIA_DECODER"],
    ("WARSHIP_DNF103P.pdf", 2): ["MCU_A", "IO", "JTAG"],
    ("WARSHIP_DNF103P.pdf", 3): ["MCU_B", "SRAM", "LCD"],
    ("WARSHIP_DNF103P.pdf", 4): [
        "ADC_DAC", "REMOTE", "WIRELESS", "TF_CARD", "ATK_MODULE",
        "TEMP_HUMI_SENSOR", "LED", "OLED_CAMERA", "LIGHT_SENSOR",
        "BEEP", "KEY", "SPI_FLASH", "TOUCH_KEY",
    ],
    ("WARSHIP_DNF103P.pdf", 5): [
        "RS232_JOYPAD", "USB_UART_USART", "RS485", "EEPROM", "RESET",
        "CAN_USB", "BOOT",
    ],
    ("WARSHIP_DNF103P.pdf", 6): [
        "DC_POWER_IN", "ON_BOARD_POWER_SOURCE", "3V3_POWER_SWITCH",
        "DVDD_POWER_1V8", "USB_UART_USB_POWER",
    ],
    ("DNF429P_F767P_H743P V2.6.pdf", 1): ["ETHERNET", "I2S_DAC_ADC", "PWM_DAC_AUDIO"],
    ("DNF429P_F767P_H743P V2.6.pdf", 2): [
        "CORE_BOARD", "IO", "RESET", "BOOT", "LCD_MCU", "VBAT", "VREF",
    ],
    ("DNF429P_F767P_H743P V2.6.pdf", 3): [
        "RS232", "WIRELESS", "OLED_CAMERA", "CAN_USB", "PRESSURE_SENSOR",
        "SD_CARD", "6_AXIS_SENSOR", "IO_EXPANDER", "TEMP_HUMI_SENSOR", "RS485",
    ],
    ("DNF429P_F767P_H743P V2.6.pdf", 4): [
        "JTAG", "LED", "REMOTE", "ALS_PS_SENSOR", "KEY", "ATK_MODULE",
        "OPTICAL_IN", "TOUCH_KEY", "BEEP", "USB_UART_USART", "ADC_DAC",
    ],
    ("DNF429P_F767P_H743P V2.6.pdf", 5): [
        "DC_POWER_IN", "3V3_POWER_SWITCH", "ANALOG_POWER",
        "USB_USART_USB_POWER", "ON_BOARD_POWER_SOURCE",
    ],
    ("STM32F429_CORE_V2.pdf", 1): ["MCU_ABC"],
    ("STM32F429_CORE_V2.pdf", 2): ["MOTHER_BOARD_CONNECTOR", "MCU_DEF"],
    ("STM32F429_CORE_V2.pdf", 3): ["MCU_GHI", "SWD", "RESET"],
    ("STM32F429_CORE_V2.pdf", 4): [
        "LDO", "TEST_POINT", "RGB_LCD", "USART1", "KEY", "LED", "TYPE_C_USB",
    ],
    ("STM32F429_CORE_V2.pdf", 5): ["SDRAM", "NAND_FLASH", "EEPROM", "SPI_FLASH"],
}


def pdf_page_count(pdf_path: Path) -> int:
    result = subprocess.run(
        ["pdfinfo", str(pdf_path)],
        check=True,
        capture_output=True,
        text=True,
        errors="replace",
    )
    for line in result.stdout.splitlines():
        if line.startswith("Pages:"):
            return int(line.split(":", 1)[1].strip())
    raise RuntimeError(f"Cannot determine page count: {pdf_path}")


def trim_white_margin(input_path: Path, output_path: Path, padding: int = 16) -> None:
    with Image.open(input_path) as source:
        image = source.convert("RGB")
        background = Image.new("RGB", image.size, "white")
        bbox = ImageChops.difference(image, background).getbbox()
        if bbox is not None:
            left = max(0, bbox[0] - padding)
            top = max(0, bbox[1] - padding)
            right = min(image.width, bbox[2] + padding)
            bottom = min(image.height, bbox[3] + padding)
            image = image.crop((left, top, right, bottom))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        image.save(output_path, "PNG", optimize=True)


def render_pdf(pdf_path: Path, output_dir: Path, page_limit: int) -> list[Path]:
    available_pages = pdf_page_count(pdf_path)
    page_count = min(page_limit, available_pages)
    with tempfile.TemporaryDirectory(prefix="schematic-") as temp_dir_name:
        temp_dir = Path(temp_dir_name)
        prefix = temp_dir / "page"
        subprocess.run(
            [
                "pdftoppm",
                "-png",
                "-r",
                "220",
                "-f",
                "1",
                "-l",
                str(page_count),
                str(pdf_path),
                str(prefix),
            ],
            check=True,
        )
        rendered_pages = sorted(temp_dir.glob("page-*.png"))
        if len(rendered_pages) != page_count:
            raise RuntimeError(
                f"Expected {page_count} pages from {pdf_path}, got {len(rendered_pages)}"
            )
        outputs = []
        for page_number, rendered_page in enumerate(rendered_pages, start=1):
            suffix = f"_p{page_number:02d}" if page_count > 1 else ""
            output_path = output_dir / f"{pdf_path.stem.replace(' ', '_')}{suffix}.png"
            trim_white_margin(rendered_page, output_path)
            outputs.append(output_path)
    return outputs


def find_red_panel_boxes(image_path: Path) -> list[tuple[int, int, int, int]]:
    encoded = np.fromfile(image_path, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Cannot read rendered image: {image_path}")

    height, width = image.shape[:2]
    blue, green, red = cv2.split(image)
    red_mask = (
        (red > 160) & (red > green * 1.35) & (red > blue * 1.35)
    ).astype(np.uint8) * 255
    red_mask = cv2.morphologyEx(
        red_mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)
    )
    contours, _ = cv2.findContours(
        red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )

    boxes = set()
    page_area = width * height
    for contour in contours:
        x, y, box_width, box_height = cv2.boundingRect(contour)
        box_area = box_width * box_height
        if not (
            box_width > 0.12 * width
            and box_height > 0.08 * height
            and 0.015 * page_area < box_area < 0.8 * page_area
        ):
            continue
        # Exclude the drawing title block at the bottom-right of a page.
        if x > 0.55 * width and y > 0.75 * height and box_height < 0.2 * height:
            continue
        boxes.add((x, y, box_width, box_height))
    return sorted(boxes, key=lambda box: (box[1], box[0]))


def extract_function_blocks(
    page_path: Path,
    titles: list[str],
    output_dir: Path,
) -> int:
    boxes = find_red_panel_boxes(page_path)
    if len(boxes) == len(titles) + 1:
        # Some sheets use a taller bottom-right drawing title block. Remove it
        # only when it is the single surplus panel expected by the title map.
        bottom_right = max(range(len(boxes)), key=lambda index: (boxes[index][1], boxes[index][0]))
        boxes.pop(bottom_right)
    if not boxes and len(titles) == 1:
        with Image.open(page_path) as image:
            image.convert("RGB").save(
                output_dir / f"{page_path.stem}_{titles[0]}.png",
                "PNG",
                optimize=True,
            )
        return 1
    if len(boxes) != len(titles):
        raise RuntimeError(
            f"Panel count mismatch for {page_path}: found {len(boxes)}, "
            f"expected {len(titles)}"
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    with Image.open(page_path) as source:
        image = source.convert("RGB")
        for title, (x, y, width, height) in zip(titles, boxes):
            padding = 8
            crop = image.crop(
                (
                    max(0, x - padding),
                    max(0, y - padding),
                    min(image.width, x + width + padding),
                    min(image.height, y + height + padding),
                )
            )
            crop.save(
                output_dir / f"{page_path.stem}_{title}.png",
                "PNG",
                optimize=True,
            )
    return len(boxes)


def main() -> None:
    if shutil.which("pdftoppm") is None or shutil.which("pdfinfo") is None:
        raise RuntimeError("Poppler tools pdftoppm and pdfinfo are required")

    generated_pages = 0
    generated_blocks = 0
    for source_name, config in BOARD_CONFIG.items():
        source_dir = SOURCE_ROOT / source_name
        if not source_dir.is_dir():
            raise FileNotFoundError(source_dir)

        board_output = OUTPUT_ROOT / config["output"]
        for pdf_path in sorted(source_dir.glob("*.pdf")):
            if pdf_path.name in config["exclude"]:
                continue
            project_pages = config["full_projects"].get(pdf_path.name)
            is_full_project = project_pages is not None
            category = "开发板" if is_full_project else "模块"
            output_pages = render_pdf(
                pdf_path,
                board_output / category,
                page_limit=project_pages if project_pages is not None else 1,
            )
            generated_pages += len(output_pages)
            if is_full_project:
                block_output = board_output / "功能电路"
                for page_number, page_path in enumerate(output_pages, start=1):
                    titles = BLOCK_TITLES[(pdf_path.name, page_number)]
                    generated_blocks += extract_function_blocks(
                        page_path, titles, block_output
                    )

    print(
        f"Generated {generated_pages} page PNGs and {generated_blocks} "
        f"function-block PNGs in {OUTPUT_ROOT}"
    )


if __name__ == "__main__":
    main()
