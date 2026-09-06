# UI

`ui/` is the architectural home for reusable human-interface components and behavior.

Examples include an OLED display implementation, reusable button handling, or indicator behavior. Prefer the concrete controller/device identity when it is known; do not invent an IC identity from the display technology alone.

Simple board-local buttons and LEDs may remain under `firmware/boards/` or `firmware/targets/` until there is reusable UI behavior worth extracting.
