v 20110115 2
T 41300 59700 9 10 1 0 0 6 1
VIN
T 41300 59300 9 10 1 0 0 6 1
GND
N 60800 51300 60800 51700 4
C 60700 51000 1 0 0 gnd-1.sym
N 57800 57300 56800 57300 4
C 56900 55500 1 0 0 gnd-1.sym
N 57000 55800 57000 56900 4
N 57000 56900 56800 56900 4
N 55200 56100 55200 57300 4
N 55200 57300 55400 57300 4
N 54900 57700 55400 57700 4
N 55400 56900 54600 56900 4
N 54600 51500 67500 51500 4
T 55600 58200 9 10 1 0 0 0 1
ICSP Header
N 53500 44900 53500 55300 4
N 53200 44300 53200 54600 4
C 67600 52900 1 90 0 resistor-1.sym
{
T 67200 53200 5 10 0 0 90 0 1
device=RESISTOR
T 67300 53100 5 10 1 1 90 0 1
refdes=R4
T 67800 53100 5 10 1 1 90 0 1
value=10k
T 67600 52900 5 10 0 0 90 0 1
footprint=0805
}
C 67300 51200 1 270 0 capacitor-1.sym
{
T 68000 51000 5 10 0 1 270 0 1
device=CAPACITOR
T 67600 50900 5 10 1 1 0 0 1
refdes=C10
T 68200 51000 5 10 0 0 270 0 1
symversion=0.1
T 67600 50400 5 10 1 1 0 0 1
value=1uF
T 67300 51200 5 10 0 0 0 0 1
footprint=0805
T 67600 50200 5 10 1 1 0 0 1
description=16V
}
C 67400 49700 1 0 0 gnd-1.sym
N 67500 51200 67500 52900 4
N 67500 53800 67500 59800 4
T 67000 40900 9 10 1 0 0 0 1
Generic I/O Base
T 66800 40600 9 10 1 0 0 0 1
mrb-gio.sch
T 67000 40300 9 10 1 0 0 0 1
1
T 68500 40300 9 10 1 0 0 0 1
1
T 70800 40300 9 10 1 0 0 0 1
Michael Petersen
C 40000 40000 0 0 0 title-bordered-D.sym
C 42200 59200 1 0 1 termblk2-1.sym
{
T 41200 59850 5 10 0 0 0 6 1
device=TERMBLK2
T 41800 60100 5 10 1 1 0 6 1
refdes=J1
T 42200 59200 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
N 61200 58400 61200 59800 4
N 60800 58400 60800 59800 4
C 63500 56200 1 0 0 gnd-1.sym
N 63600 56500 63600 56700 4
N 63600 56700 63400 56700 4
N 47300 59800 67500 59800 4
N 63400 57000 64300 57000 4
N 63700 57000 63700 59800 4
C 64200 55700 1 0 0 gnd-1.sym
N 64300 56900 64300 57000 4
C 64900 56900 1 270 0 capacitor-1.sym
{
T 65600 56700 5 10 0 1 270 0 1
device=CAPACITOR
T 65200 56600 5 10 1 1 0 0 1
refdes=C7
T 65800 56700 5 10 0 0 270 0 1
symversion=0.1
T 65200 56100 5 10 1 1 0 0 1
value=0.1uF
T 64900 56900 5 10 0 0 0 0 1
footprint=0805
T 65200 55900 5 10 1 1 0 0 1
description=16V
}
C 65000 55700 1 0 0 gnd-1.sym
N 65100 56900 65100 57300 4
N 65100 57300 63400 57300 4
C 64700 58100 1 0 1 gnd-1.sym
N 64600 59300 64600 59800 4
N 63400 52500 67500 52500 4
C 57500 50700 1 0 0 crystal-1.sym
{
T 57700 51200 5 10 0 0 0 0 1
device=CRYSTAL
T 57700 51000 5 10 1 1 0 0 1
refdes=Y1
T 57700 51400 5 10 0 0 0 0 1
symversion=0.1
T 58350 50600 5 10 1 1 0 0 1
value=20MHz
T 57500 50700 5 10 0 1 0 0 1
footprint=crystal-hc49
}
N 57500 50300 57500 55800 4
N 58200 50300 58200 55500 4
C 57400 49100 1 0 0 gnd-1.sym
C 58000 50300 1 270 0 capacitor-1.sym
{
T 58700 50100 5 10 0 1 270 0 1
device=CAPACITOR
T 58300 50000 5 10 1 1 0 0 1
refdes=C9
T 58900 50100 5 10 0 0 270 0 1
symversion=0.1
T 58300 49500 5 10 1 1 0 0 1
value=22pF
T 58000 50300 5 10 0 0 0 0 1
footprint=0805
T 58300 49300 5 10 1 1 0 0 1
description=16V, NP0
}
C 58100 49100 1 0 0 gnd-1.sym
N 58200 55500 58600 55500 4
N 58600 55800 57500 55800 4
N 61200 51300 61200 51700 4
C 61100 51000 1 0 0 gnd-1.sym
N 57800 56700 57800 57300 4
N 54600 56900 54600 51500 4
N 47100 54600 58600 54600 4
N 58600 54300 53500 54300 4
N 55200 56100 58600 56100 4
N 54900 56400 58600 56400 4
N 54900 56400 54900 57700 4
N 57800 56700 58600 56700 4
N 52000 54000 58600 54000 4
N 52500 53700 58600 53700 4
C 42300 58800 1 0 0 gnd-1.sym
N 42400 59100 42400 59400 4
N 42400 59400 42200 59400 4
N 53800 44600 53800 54000 4
C 65600 50200 1 90 0 resistor-1.sym
{
T 65200 50500 5 10 0 0 90 0 1
device=RESISTOR
T 65300 50400 5 10 1 1 90 0 1
refdes=R3
T 65800 50400 5 10 1 1 90 0 1
value=1Meg
T 65600 50200 5 10 0 0 90 0 1
footprint=0805
T 65600 50100 5 10 1 1 0 0 1
description=1%
}
C 66000 54700 1 0 0 resistor-1.sym
{
T 66300 55100 5 10 0 0 0 0 1
device=RESISTOR
T 66200 55000 5 10 1 1 0 0 1
refdes=R2
T 66200 54500 5 10 1 1 0 0 1
value=4.99Meg
T 66000 54700 5 10 0 0 0 0 1
footprint=0805
T 66700 54900 5 10 1 1 0 0 1
description=1%
}
N 65500 51100 65500 55500 4
N 66900 54800 68000 54800 4
N 68000 54800 68000 61200 4
N 68000 61200 44400 61200 4
C 65400 49700 1 0 0 gnd-1.sym
N 65500 50000 65500 50200 4
N 67500 50000 67500 50300 4
N 44800 57200 51000 57200 4
N 44800 59000 44800 59800 4
N 42200 59800 43000 59800 4
{
T 42400 59800 5 10 1 1 0 0 1
netname=VIN
}
N 44400 61200 44400 59800 4
C 55400 56700 1 0 0 avrprog-1.sym
{
T 55400 58300 5 10 0 1 0 0 1
device=AVRPROG
T 56000 58000 5 10 1 1 0 0 1
refdes=J2
T 55600 56500 5 10 0 1 0 0 1
footprint=JUMPER3x2
}
T 41000 41800 9 10 1 0 0 2 3
Notes:
1) All capacitors are ceramic (X7R/X5R) unless otherwise noted.
2) All capacitors and resistors are 0805 unless otherwise noted.
C 43000 59600 1 0 0 schottky-1.sym
{
T 43322 60272 5 10 0 0 0 0 1
device=DIODE
T 43300 60100 5 10 1 1 0 0 1
refdes=D5
T 43341 60432 5 10 0 1 0 0 1
footprint=SOD123
T 42800 59300 5 10 1 1 0 0 1
model=MBR0540LT1G
}
N 43900 59800 45700 59800 4
N 44800 57200 44800 58100 4
C 44600 59000 1 270 0 Cap_H-2.sym
{
T 44900 58800 5 10 1 1 0 0 1
refdes=C1
T 46100 59000 5 10 0 0 270 0 1
device=Capacitor
T 44900 58300 5 10 1 1 0 2 1
value=68uF
T 44600 59000 5 10 0 0 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
T 44900 57900 5 10 1 1 0 0 1
description=25V, Electrolytic
}
N 56800 57700 57000 57700 4
N 57000 57700 57000 59800 4
C 57700 50300 1 90 1 capacitor-1.sym
{
T 57000 50100 5 10 0 1 270 2 1
device=CAPACITOR
T 57400 50000 5 10 1 1 0 6 1
refdes=C8
T 56800 50100 5 10 0 0 270 2 1
symversion=0.1
T 57400 49500 5 10 1 1 0 6 1
value=22pF
T 57700 50300 5 10 0 0 0 6 1
footprint=0805
T 57400 49300 5 10 1 1 0 6 1
description=16V, NP0
}
C 64100 56900 1 270 0 capacitor-1.sym
{
T 64800 56700 5 10 0 1 270 0 1
device=CAPACITOR
T 64400 56600 5 10 1 1 0 0 1
refdes=C6
T 65000 56700 5 10 0 0 270 0 1
symversion=0.1
T 64400 56100 5 10 1 1 0 0 1
value=0.1uF
T 64100 56900 5 10 0 0 0 0 1
footprint=0805
T 64400 55900 5 10 1 1 0 0 1
description=16V
}
C 64800 59300 1 90 1 capacitor-1.sym
{
T 64100 59100 5 10 0 1 270 2 1
device=CAPACITOR
T 64500 59000 5 10 1 1 0 6 1
refdes=C4
T 63900 59100 5 10 0 0 270 2 1
symversion=0.1
T 64500 58500 5 10 1 1 0 6 1
value=0.1uF
T 64800 59300 5 10 0 0 0 6 1
footprint=0805
T 64500 58300 5 10 1 1 0 6 1
description=16V
}
N 66000 54800 65500 54800 4
N 65500 55500 63400 55500 4
T 66600 54400 9 10 1 0 0 5 1
(MRBW: 1k)
T 67000 49500 9 10 1 0 180 0 1
(MRBW: not populated)
N 46500 59200 46500 57200 4
C 50900 56700 1 0 0 gnd-1.sym
N 48000 58700 48000 59800 4
N 48000 57800 48000 57200 4
C 51100 58600 1 90 0 resistor-1.sym
{
T 50700 58900 5 10 0 0 90 0 1
device=RESISTOR
T 50800 58800 5 10 1 1 90 0 1
refdes=R1
T 51300 58800 5 10 1 1 90 0 1
value=330
T 51100 58600 5 10 0 0 90 0 1
footprint=0805
}
C 51200 57500 1 90 0 led-3.sym
{
T 51450 57450 5 10 1 1 90 0 1
device=GREEN LED
T 50650 57950 5 10 1 1 90 0 1
refdes=D1
T 51200 57500 5 10 0 0 0 0 1
footprint=1206
}
N 51000 58600 51000 58400 4
N 51000 59500 51000 59800 4
N 51000 57000 51000 57500 4
C 47800 58700 1 270 0 capacitor-1.sym
{
T 48500 58500 5 10 0 1 270 0 1
device=CAPACITOR
T 48100 58400 5 10 1 1 0 0 1
refdes=C2
T 48700 58500 5 10 0 0 270 0 1
symversion=0.1
T 48100 57900 5 10 1 1 0 0 1
value=1uF
T 48100 57500 5 10 0 1 0 0 1
footprint=0805
T 48100 57700 5 10 1 1 0 0 1
comment=16V
}
C 45000 54700 1 270 1 mosfet-with-diode-1.sym
{
T 45500 55600 5 10 0 0 90 2 1
device=NPN_TRANSISTOR
T 45000 55600 5 10 1 1 180 6 1
refdes=Q1
T 45000 55800 5 10 1 1 180 6 1
value=BSS138
T 45000 54700 5 10 0 0 180 6 1
footprint=SOT23_MOS
}
C 43600 50100 1 0 0 xbee-1.sym
{
T 46000 54500 5 10 0 0 0 0 1
device=XBEE
T 44800 53500 5 10 1 1 0 3 1
refdes=XU4
T 43600 50100 5 10 0 1 270 0 1
footprint=XBEE
}
C 49000 51400 1 0 0 resistor-1.sym
{
T 49300 51800 5 10 0 0 0 0 1
device=RESISTOR
T 49200 51700 5 10 1 1 0 0 1
refdes=R9
T 49200 51200 5 10 1 1 0 0 1
value=330
T 49000 51400 5 10 0 0 0 0 1
footprint=0805
}
C 50500 50500 1 90 0 led-3.sym
{
T 50750 50250 5 10 1 1 90 0 1
device=RED (ASSOC) LED
T 49950 50950 5 10 1 1 90 0 1
refdes=D3
T 50500 50500 5 10 0 0 0 0 1
footprint=1206
}
N 49000 51500 46000 51500 4
C 42000 51400 1 0 0 resistor-1.sym
{
T 42300 51800 5 10 0 0 0 0 1
device=RESISTOR
T 42200 51700 5 10 1 1 0 0 1
refdes=R8
T 42200 51200 5 10 1 1 0 0 1
value=330
T 42000 51400 5 10 0 0 0 0 1
footprint=0805
}
C 42000 50500 1 90 0 led-3.sym
{
T 41350 50050 5 10 1 1 90 0 1
device=YELLOW (RSSI) LED
T 42250 50850 5 10 1 1 90 0 1
refdes=D4
T 42000 50500 5 10 0 0 0 0 1
footprint=1206
}
N 42900 51500 43600 51500 4
C 41700 50000 1 0 0 gnd-1.sym
N 41800 50300 43600 50300 4
C 50200 50000 1 0 0 gnd-1.sym
C 46100 54000 1 270 1 mosfet-with-diode-1.sym
{
T 46600 54900 5 10 0 0 90 2 1
device=NPN_TRANSISTOR
T 46100 54900 5 10 1 1 180 6 1
refdes=Q2
T 46100 55100 5 10 1 1 180 6 1
value=BSS138
T 46100 54000 5 10 0 0 180 6 1
footprint=SOT23_MOS
}
N 41800 51400 41800 51500 4
N 41800 51500 42000 51500 4
N 50300 51400 50300 51500 4
N 50300 51500 49900 51500 4
N 41800 50500 41800 50300 4
N 50300 50300 50300 50500 4
N 43600 52700 42400 52700 4
N 42000 55300 45000 55300 4
N 43600 52400 42000 52400 4
N 42000 52400 42000 55300 4
N 42400 52700 42400 54600 4
N 42400 54600 46100 54600 4
C 43200 53400 1 90 0 resistor-1.sym
{
T 42800 53700 5 10 0 0 90 0 1
device=RESISTOR
T 42900 53400 5 10 1 1 90 0 1
refdes=R11
T 42900 53900 5 10 1 1 90 0 1
value=10k
T 43200 53400 5 10 0 0 90 0 1
footprint=0805
}
N 43100 49500 43100 53400 4
N 43100 53000 43600 53000 4
N 43400 53000 43400 54000 4
N 43400 54000 49200 54000 4
N 43100 54300 43100 55300 4
N 45500 54700 45500 54000 4
C 49900 52400 1 0 0 gnd-1.sym
{
T 49900 52400 5 10 0 0 0 0 1
netname=GND
}
N 50000 52700 50000 53400 4
C 46100 53800 1 270 0 capacitor-1.sym
{
T 46800 53600 5 10 0 1 270 0 1
device=CAPACITOR
T 46400 53500 5 10 1 1 0 0 1
refdes=C15
T 47000 53600 5 10 0 0 270 0 1
symversion=0.1
T 46400 53000 5 10 1 1 0 0 1
value=1uF
T 46100 53800 5 10 0 0 0 0 1
footprint=0805
T 46400 52800 5 10 1 1 0 0 1
description=16V
}
C 46800 53800 1 270 0 capacitor-1.sym
{
T 47500 53600 5 10 0 1 270 0 1
device=CAPACITOR
T 47100 53500 5 10 1 1 0 0 1
refdes=C14
T 47700 53600 5 10 0 0 270 0 1
symversion=0.1
T 47100 53000 5 10 1 1 0 0 1
value=8.2pF
T 46800 53800 5 10 0 0 0 0 1
footprint=0805
T 47100 52800 5 10 1 1 0 0 1
description=16V, NP0
}
N 46300 53800 46300 54000 4
N 47000 53800 47000 54000 4
N 47000 52900 47000 52700 4
N 46300 52700 50800 52700 4
N 46300 52900 46300 52700 4
C 47700 56000 1 180 0 resistor-1.sym
{
T 47400 55600 5 10 0 0 180 0 1
device=RESISTOR
T 47700 55700 5 10 1 1 180 0 1
refdes=R10
T 47200 55700 5 10 1 1 180 0 1
value=10k
T 47700 56000 5 10 0 0 180 0 1
footprint=0805
}
C 46200 55900 1 0 0 5V-plus-1.sym
{
T 46200 55900 5 10 0 1 0 0 1
netname=+5V
}
N 46400 55900 46800 55900 4
N 47700 55900 48100 55900 4
N 48100 55900 48100 54600 4
C 50800 53400 1 0 1 tlv1117-1.sym
{
T 50400 54700 5 10 1 1 0 6 1
device=TLV1117-33
T 49800 54900 5 10 1 1 0 0 1
refdes=U5
T 50800 53400 5 10 0 0 0 0 1
footprint=SOT223
}
N 49000 54000 49000 54200 4
N 49000 54200 49200 54200 4
C 48100 53800 1 270 0 capacitor-1.sym
{
T 48800 53600 5 10 0 1 270 0 1
device=CAPACITOR
T 48400 53500 5 10 1 1 0 0 1
refdes=C13
T 49000 53600 5 10 0 0 270 0 1
symversion=0.1
T 48400 53000 5 10 1 1 0 0 1
value=22uF
T 48100 53800 5 10 0 0 0 0 1
footprint=0805
T 48400 52800 5 10 1 1 0 0 1
description=16V, Tantalum
}
N 48300 53800 48300 54000 4
N 48300 52900 48300 52700 4
C 50800 54100 1 0 0 5V-plus-1.sym
{
T 50800 54100 5 10 0 1 0 0 1
netname=+5V
}
C 50600 53800 1 270 0 capacitor-1.sym
{
T 51300 53600 5 10 0 1 270 0 1
device=CAPACITOR
T 50400 53500 5 10 1 1 0 0 1
refdes=C12
T 51500 53600 5 10 0 0 270 0 1
symversion=0.1
T 50300 53000 5 10 1 1 0 0 1
value=10uF
T 50600 53800 5 10 0 0 0 0 1
footprint=0805
T 50400 52800 5 10 1 1 0 0 1
description=16V
}
N 51000 54000 51000 54100 4
N 50800 52700 50800 52900 4
N 50800 53800 50800 54000 4
N 50800 54000 51000 54000 4
T 48900 52400 9 10 1 0 0 6 1
Place C11 & C12 near XBee pin 1
T 50700 56000 9 10 1 0 0 6 1
MRBW (Wireless)
N 46000 55300 53500 55300 4
B 41000 48000 10500 8500 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
C 47200 50100 1 270 0 mosfet-with-diode-1.sym
{
T 47700 49200 5 10 0 0 270 0 1
device=NPN_TRANSISTOR
T 47200 49200 5 10 1 1 0 0 1
refdes=Q3
T 47200 49000 5 10 1 1 0 0 1
value=BSS138
T 47200 50100 5 10 0 0 0 0 1
footprint=SOT23_MOS
}
C 48000 49300 1 270 0 mosfet-with-diode-1.sym
{
T 48500 48400 5 10 0 0 270 0 1
device=NPN_TRANSISTOR
T 48000 48400 5 10 1 1 0 0 1
refdes=Q4
T 48000 48200 5 10 1 1 0 0 1
value=BSS138
T 48000 49300 5 10 0 0 0 0 1
footprint=SOT23_MOS
}
C 47500 60000 1 0 0 5V-plus-1.sym
{
T 47500 60000 5 10 0 0 0 0 1
netname=+5V
}
N 47700 60000 47700 59800 4
C 47300 54100 1 0 0 3V3-plus-1.sym
N 47500 54100 47500 54000 4
N 46000 51800 46800 51800 4
N 46000 50600 46500 50600 4
N 46500 48700 46500 50600 4
N 46500 48700 48000 48700 4
N 45200 49500 47200 49500 4
N 46800 49500 46800 51800 4
C 47100 51800 1 0 0 3V3-plus-1.sym
N 47300 51800 47300 50900 4
N 46000 50900 48500 50900 4
N 47700 50100 47700 50900 4
N 48500 50900 48500 49300 4
N 52000 49500 52000 54000 4
N 52000 49500 48200 49500 4
N 52500 53700 52500 48700 4
N 52500 48700 49000 48700 4
C 40800 60700 1 0 0 hole-1.sym
{
T 40800 60700 5 10 0 1 0 0 1
device=HOLE
T 41000 61300 5 10 1 1 0 4 1
refdes=H1
T 40800 60700 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 41300 60700 1 0 0 hole-1.sym
{
T 41300 60700 5 10 0 1 0 0 1
device=HOLE
T 41500 61300 5 10 1 1 0 4 1
refdes=H2
T 41300 60700 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 41800 60700 1 0 0 hole-1.sym
{
T 41800 60700 5 10 0 1 0 0 1
device=HOLE
T 42000 61300 5 10 1 1 0 4 1
refdes=H3
T 41800 60700 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 42300 60700 1 0 0 hole-1.sym
{
T 42300 60700 5 10 0 1 0 0 1
device=HOLE
T 42500 61300 5 10 1 1 0 4 1
refdes=H4
T 42300 60700 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
N 43500 47200 46700 47200 4
C 45300 43600 1 0 1 rs485-1.sym
{
T 43650 45400 5 10 0 0 0 6 1
device=MAX489
T 43950 43750 5 10 1 1 0 6 1
refdes=XU3
T 43650 45200 5 10 0 0 0 6 1
footprint=DIP8
}
N 45300 44900 53500 44900 4
N 45300 44600 53800 44600 4
N 44500 42900 46200 42900 4
N 45500 42900 45500 44000 4
N 45500 44000 45300 44000 4
C 46800 46100 1 90 0 resistor-1.sym
{
T 46400 46400 5 10 0 0 90 0 1
device=RESISTOR
T 46500 46300 5 10 1 1 90 0 1
refdes=R7
T 47000 46300 5 10 1 1 90 0 1
value=330
T 46800 46100 5 10 0 0 90 0 1
footprint=0805
}
C 46900 45100 1 90 0 led-3.sym
{
T 47150 45050 5 10 1 1 90 0 1
device=AMBER LED
T 46350 45550 5 10 1 1 90 0 1
refdes=D2
T 46900 45100 5 10 0 0 0 0 1
footprint=1206
}
N 46700 46100 46700 46000 4
C 45800 45100 1 90 0 resistor-1.sym
{
T 45400 45400 5 10 0 0 90 0 1
device=RESISTOR
T 45500 45300 5 10 1 1 90 0 1
refdes=R5
T 46000 45300 5 10 1 1 90 0 1
value=10k
T 45800 45100 5 10 0 0 90 0 1
footprint=0805
}
C 46300 43100 1 90 0 resistor-1.sym
{
T 45900 43400 5 10 0 0 90 0 1
device=RESISTOR
T 46000 43300 5 10 1 1 90 0 1
refdes=R6
T 46500 43300 5 10 1 1 90 0 1
value=10k
T 46300 43100 5 10 0 0 90 0 1
footprint=0805
}
N 45700 45100 45700 44900 4
N 44500 42600 44500 43600 4
N 44500 45500 44500 47200 4
N 45700 47200 45700 46000 4
N 46700 45100 46700 44300 4
N 45300 44300 53200 44300 4
N 46200 44000 46200 44600 4
N 46200 42900 46200 43100 4
N 46700 47000 46700 47200 4
C 43000 44400 1 0 1 termblk2-1.sym
{
T 42000 45050 5 10 0 0 0 6 1
device=TERMBLK2
T 42600 44200 5 10 1 1 0 6 1
refdes=J3
T 43000 44400 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
N 43000 44600 43200 44600 4
N 43200 44600 43200 44200 4
N 43200 44200 43700 44200 4
N 43000 45000 43700 45000 4
C 44400 42300 1 0 0 gnd-1.sym
C 43400 45800 1 0 0 gnd-1.sym
C 43300 47000 1 270 0 capacitor-1.sym
{
T 44000 46800 5 10 0 1 270 0 1
device=CAPACITOR
T 43600 46700 5 10 1 1 0 0 1
refdes=C11
T 44200 46800 5 10 0 0 270 0 1
symversion=0.1
T 43600 46200 5 10 1 1 0 0 1
value=0.1uF
T 43300 47000 5 10 0 0 0 0 1
footprint=0805
T 43600 46000 5 10 1 1 0 0 1
description=16V
}
N 43500 47000 43500 47200 4
B 41000 42000 6500 5500 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
T 42100 44900 9 10 1 0 0 6 1
RS485-A
T 42100 44500 9 10 1 0 0 6 1
RS485-B
T 41200 47100 9 10 1 0 0 0 1
MRB (Wired)
C 49000 49800 1 0 0 5V-plus-1.sym
{
T 49000 49800 5 10 0 1 0 0 1
netname=+5V
}
C 49400 48900 1 0 0 resistor-1.sym
{
T 49700 49300 5 10 0 0 0 0 1
device=RESISTOR
T 49400 49200 5 10 1 1 0 0 1
refdes=R13
T 49900 49200 5 10 1 1 0 0 1
value=10k
T 49400 48900 5 10 0 0 0 0 1
footprint=0805
}
N 50300 49000 50500 49000 4
N 50500 49000 50500 48700 4
N 49200 49800 49200 49000 4
N 49200 49000 49400 49000 4
C 45200 49600 1 180 0 resistor-1.sym
{
T 44900 49200 5 10 0 0 180 0 1
device=RESISTOR
T 45200 49300 5 10 1 1 180 0 1
refdes=R12
T 44700 49300 5 10 1 1 180 0 1
value=10k
T 45200 49600 5 10 0 0 180 0 1
footprint=0805
}
N 43100 49500 44300 49500 4
N 58600 53400 55000 53400 4
N 55000 53400 55000 47200 4
C 55600 47200 1 90 1 termblk2-1.sym
{
T 54950 46200 5 10 0 0 90 6 1
device=TERMBLK2
T 55200 46400 5 10 1 1 0 5 1
refdes=Jx
T 55600 47200 5 10 0 1 90 0 1
footprint=TERMBLK2_200MIL
}
C 56400 47200 1 90 1 termblk2-1.sym
{
T 55750 46200 5 10 0 0 90 6 1
device=TERMBLK2
T 56000 46400 5 10 1 1 0 5 1
refdes=Jx
T 56400 47200 5 10 0 1 90 0 1
footprint=TERMBLK2_200MIL
}
N 56200 47200 56200 52500 4
N 56200 52500 58600 52500 4
N 58600 52800 55800 52800 4
N 55800 52800 55800 47200 4
N 55400 47200 55400 53100 4
N 55400 53100 58600 53100 4
C 57200 47200 1 90 1 termblk2-1.sym
{
T 56550 46200 5 10 0 0 90 6 1
device=TERMBLK2
T 56800 46400 5 10 1 1 0 5 1
refdes=Jx
T 57200 47200 5 10 0 1 90 0 1
footprint=TERMBLK2_200MIL
}
C 58000 47200 1 90 1 termblk2-1.sym
{
T 57350 46200 5 10 0 0 90 6 1
device=TERMBLK2
T 57600 46400 5 10 1 1 0 5 1
refdes=Jx
T 58000 47200 5 10 0 1 90 0 1
footprint=TERMBLK2_200MIL
}
N 56600 47200 56600 48500 4
N 56600 48500 64000 48500 4
N 64000 48500 64000 54300 4
N 64000 54300 63400 54300 4
N 63400 54000 64300 54000 4
N 64300 54000 64300 48200 4
N 57000 48200 64300 48200 4
N 57000 48200 57000 47200 4
N 57400 47200 57400 47900 4
N 57400 47900 64600 47900 4
N 64600 47900 64600 53700 4
N 64600 53700 63400 53700 4
N 63400 53400 64900 53400 4
N 64900 47600 64900 53400 4
N 57800 47600 64900 47600 4
N 57800 47600 57800 47200 4
C 58600 51700 1 0 0 mega328-tqfp32.sym
{
T 63100 58200 5 10 1 1 0 6 1
refdes=U2
T 58900 58500 5 10 0 0 0 0 1
device=ATMega328-TQFP32
T 58900 58700 5 10 0 0 0 0 1
footprint=TQFP32_7
}
C 65600 58100 1 0 1 gnd-1.sym
C 65700 59300 1 90 1 capacitor-1.sym
{
T 65000 59100 5 10 0 1 270 2 1
device=CAPACITOR
T 65400 59000 5 10 1 1 0 6 1
refdes=C5
T 64800 59100 5 10 0 0 270 2 1
symversion=0.1
T 65400 58500 5 10 1 1 0 6 1
value=0.1uF
T 65700 59300 5 10 0 0 0 6 1
footprint=0805
T 65400 58300 5 10 1 1 0 6 1
description=16V
}
N 65500 59300 65500 59800 4
C 45700 59200 1 0 0 lm7805-1.sym
{
T 47100 60200 5 10 1 1 0 6 1
refdes=U1
T 45700 59200 5 10 0 1 0 0 1
footprint=RECOM-TO220
T 46600 59000 5 10 1 1 0 0 1
device=R-78E-5.0-0.5
}
C 48800 58700 1 270 0 capacitor-1.sym
{
T 49500 58500 5 10 0 1 270 0 1
device=CAPACITOR
T 49100 58400 5 10 1 1 0 0 1
refdes=C3
T 49700 58500 5 10 0 0 270 0 1
symversion=0.1
T 49100 57900 5 10 1 1 0 0 1
value=22uF
T 48800 58700 5 10 0 0 0 0 1
footprint=0805
T 49100 57700 5 10 1 1 0 0 1
description=16V, Tantalum
}
N 49000 57800 49000 57200 4
N 49000 58700 49000 59800 4
C 44800 46500 1 0 0 5V-plus-1.sym
{
T 44800 46500 5 10 0 1 0 0 1
netname=+5V
}
N 45000 46500 45000 46300 4
N 45000 46300 44500 46300 4
C 55800 44900 1 90 1 termblk2-1.sym
{
T 55150 43900 5 10 0 0 90 6 1
device=TERMBLK2
T 55400 44100 5 10 1 1 0 5 1
refdes=Jx
T 55800 44900 5 10 0 1 90 0 1
footprint=TERMBLK2_200MIL
}
C 57400 44900 1 90 1 termblk2-1.sym
{
T 56750 43900 5 10 0 0 90 6 1
device=TERMBLK2
T 57000 44100 5 10 1 1 0 5 1
refdes=Jx
T 57400 44900 5 10 0 1 90 0 1
footprint=TERMBLK2_200MIL
}
C 55000 45200 1 0 0 5V-plus-1.sym
{
T 55000 45200 5 10 0 1 0 0 1
netname=+5V
}
N 55200 45200 55200 44900 4
N 55600 44900 55600 45100 4
N 55600 45100 55200 45100 4
C 57700 44600 1 0 0 gnd-1.sym
N 56800 44900 56800 45100 4
N 56800 45100 57800 45100 4
N 57200 44900 57200 45100 4
N 57800 45100 57800 44900 4
