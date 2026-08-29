# 2026-08-27 — Initial Product Research

## Time spent: 45 mins

## What I worked on

I want to create a very thin, portable PCB keychain with an integrated display and backlighting. Rather than making another generic macropad or a PCB where the LEDs are the main feature, I want the PCB itself to function as a small collectible digital gadget.

- A small XIAO RP2040 as the microcontroller
- A thin e-paper display integrated into the front of the PCB (Wave share 2.13 inch e ink display) 
- Two small buttons on the rear for changing the displayed artwork
- Four individually addressable RGB LEDs on the rear of the PCB
- A small battery
- The entire electronics assembly integrated into a compact keychain-shaped PCB
<img width="1059" height="749" alt="image" src="https://github.com/user-attachments/assets/795ad6b0-a746-4871-b309-4ee80042259d" />

# 2026-08-28 — Schematic Research and Design

## Time spent: 2hr 15mins

Today I began creating the schematic for the PCB in KiCad. 

Problems:
- the waveshare eink display uses a zif connector so i was unsure as to how to create the connection in Kicad as i had never dealt with ribbon cable.
- I decided to switch to a normal tft display as ii had more experience with those and I could more likely solder it.
- I was unsure as to how to show charging indication on such a small pcb and organise componanats
- I was unsure as to how to power the chosen display whiilst efficently using space

Fix:
- I found about male connector pins in kicad and realised i could assign a fpc footprint to them allowing me to keep the e ink display as it was more power effcient
- For the charging indicators I added a 4 very small onboard leds which will. also be user customisable
- I decided to go with a lipo battery for easy charging
<img width="1059" height="749" alt="image" src="https://github.com/user-attachments/assets/a3f1de75-bd9b-458c-837d-ce02dd3c7803" />
