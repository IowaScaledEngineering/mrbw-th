v 20110115 2
T 41300 59700 9 10 1 0 0 6 1
VIN
T 41300 59300 9 10 1 0 0 6 1
GND
N 66400 51300 66400 51700 4
C 66300 51000 1 0 0 gnd-1.sym
N 63400 57300 62400 57300 4
C 62500 55500 1 0 0 gnd-1.sym
N 62600 55800 62600 56900 4
N 62600 56900 62400 56900 4
N 60800 56100 60800 57300 4
N 60800 57300 61000 57300 4
N 60500 57700 61000 57700 4
N 61000 56900 60200 56900 4
N 60200 51500 71500 51500 4
T 61200 58200 9 10 1 0 0 0 1
ICSP Header
C 67000 43500 1 0 0 gnd-1.sym
N 61200 44400 61200 54300 4
N 60900 43800 60900 54600 4
C 71600 52900 1 90 0 resistor-1.sym
{
T 71200 53200 5 10 0 0 90 0 1
device=RESISTOR
T 71300 53100 5 10 1 1 90 0 1
refdes=R5
T 71800 53100 5 10 1 1 90 0 1
value=10k
T 71600 52900 5 10 0 0 90 0 1
footprint=0805
}
C 71300 51200 1 270 0 capacitor-1.sym
{
T 72000 51000 5 10 0 1 270 0 1
device=CAPACITOR
T 71600 50900 5 10 1 1 0 0 1
refdes=C10
T 72200 51000 5 10 0 0 270 0 1
symversion=0.1
T 71600 50400 5 10 1 1 0 0 1
value=1uF
T 71300 51200 5 10 0 0 0 0 1
footprint=0805
T 71600 50200 5 10 1 1 0 0 1
description=16V
}
C 71400 49700 1 0 0 gnd-1.sym
N 71500 51200 71500 52900 4
N 71500 53800 71500 59200 4
T 72200 51500 9 10 1 0 90 3 1
Optional (improved noise immunity)
T 67000 40900 9 10 1 0 0 0 1
Wireless MRBus Temperature/Humidity Sensor
T 66800 40600 9 10 1 0 0 0 1
mrbw-th.sch
T 67000 40300 9 10 1 0 0 0 1
1
T 68500 40300 9 10 1 0 0 0 1
1
T 70800 40300 9 10 1 0 0 0 1
Michael Petersen
C 40000 40000 0 0 0 title-bordered-D.sym
N 66200 45200 67400 45200 4
C 42200 59200 1 0 1 termblk2-1.sym
{
T 41200 59850 5 10 0 0 0 6 1
device=TERMBLK2
T 41800 60100 5 10 1 1 0 6 1
refdes=J1
T 42200 59200 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
C 42800 42200 1 0 0 hole-1.sym
{
T 42800 42200 5 10 0 1 0 0 1
device=HOLE
T 43000 42800 5 10 1 1 0 4 1
refdes=H1
T 42800 42200 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 43300 42200 1 0 0 hole-1.sym
{
T 43300 42200 5 10 0 1 0 0 1
device=HOLE
T 43500 42800 5 10 1 1 0 4 1
refdes=H2
T 43300 42200 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 43800 42200 1 0 0 hole-1.sym
{
T 43800 42200 5 10 0 1 0 0 1
device=HOLE
T 44000 42800 5 10 1 1 0 4 1
refdes=H3
T 43800 42200 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 44300 42200 1 0 0 hole-1.sym
{
T 44300 42200 5 10 0 1 0 0 1
device=HOLE
T 44500 42800 5 10 1 1 0 4 1
refdes=H4
T 44300 42200 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 64200 51700 1 0 0 mega48-tqfp32.sym
{
T 68700 58200 5 10 1 1 0 6 1
refdes=U2
T 64500 58500 5 10 0 0 0 0 1
device=ATMega48-TQFP32
T 68000 51800 5 10 1 1 0 0 1
footprint=TQFP32
}
N 66800 58400 66800 59200 4
N 66400 58400 66400 59200 4
C 69100 56200 1 0 0 gnd-1.sym
N 69200 56500 69200 56700 4
N 69200 56700 69000 56700 4
N 49900 59200 71500 59200 4
{
T 63900 59200 5 10 1 1 0 0 1
netname=VDD
}
N 69000 57000 69900 57000 4
N 69300 57000 69300 59200 4
C 69800 55500 1 0 0 gnd-1.sym
N 69900 56700 69900 57000 4
C 70500 56700 1 270 0 capacitor-1.sym
{
T 71200 56500 5 10 0 1 270 0 1
device=CAPACITOR
T 70800 56400 5 10 1 1 0 0 1
refdes=C7
T 71400 56500 5 10 0 0 270 0 1
symversion=0.1
T 70800 55900 5 10 1 1 0 0 1
value=0.1uF
T 70500 56700 5 10 0 0 0 0 1
footprint=0805
T 70800 55700 5 10 1 1 0 0 1
description=16V
}
C 70600 55500 1 0 0 gnd-1.sym
N 70700 56700 70700 57300 4
N 70700 57300 69000 57300 4
C 70300 57600 1 0 1 gnd-1.sym
N 70200 58800 70200 59200 4
N 69000 52500 71500 52500 4
C 63100 50700 1 0 0 crystal-1.sym
{
T 63300 51200 5 10 0 0 0 0 1
device=CRYSTAL
T 63300 51000 5 10 1 1 0 0 1
refdes=Y1
T 63300 51400 5 10 0 0 0 0 1
symversion=0.1
T 63950 50600 5 10 1 1 0 0 1
value=11.0592MHz
T 63100 50700 5 10 0 1 0 0 1
footprint=crystal-hc49
}
N 63100 50300 63100 55800 4
N 63800 50300 63800 55500 4
C 63000 49100 1 0 0 gnd-1.sym
C 63600 50300 1 270 0 capacitor-1.sym
{
T 64300 50100 5 10 0 1 270 0 1
device=CAPACITOR
T 63900 50000 5 10 1 1 0 0 1
refdes=C9
T 64500 50100 5 10 0 0 270 0 1
symversion=0.1
T 63900 49500 5 10 1 1 0 0 1
value=33pF
T 63600 50300 5 10 0 0 0 0 1
footprint=0805
T 63900 49300 5 10 1 1 0 0 1
description=16V, NP0
}
C 63700 49100 1 0 0 gnd-1.sym
N 63800 55500 64200 55500 4
N 64200 55800 63100 55800 4
N 66800 51300 66800 51700 4
C 66700 51000 1 0 0 gnd-1.sym
N 63400 53200 63400 57300 4
N 60200 56900 60200 51500 4
N 64200 54600 60900 54600 4
N 64200 54300 61200 54300 4
N 58900 56100 64200 56100 4
N 58500 56400 64200 56400 4
N 60500 56400 60500 57700 4
N 63400 56700 64200 56700 4
N 61500 54000 64200 54000 4
N 61800 53700 64200 53700 4
N 59600 46700 59600 59200 4
N 72000 45200 69800 45200 4
C 42300 58800 1 0 0 gnd-1.sym
N 42400 59100 42400 59400 4
N 42400 59400 42200 59400 4
C 67400 43800 1 0 0 xbee-1.sym
{
T 69800 48200 5 10 0 0 0 0 1
device=XBEE
T 68600 47200 5 10 1 1 0 3 1
refdes=XU5
T 67400 43800 5 10 0 1 270 0 1
footprint=XBEE
}
N 67100 43800 67100 44000 4
N 67100 44000 67400 44000 4
N 67400 46400 60900 46400 4
N 61200 46100 67400 46100 4
N 70200 45500 70200 48000 4
N 70200 48000 61500 48000 4
N 61500 44100 61500 54000 4
N 61800 53700 61800 48300 4
N 61800 48300 70500 48300 4
N 70500 44300 70500 48300 4
N 70500 44300 69800 44300 4
N 57000 46700 67400 46700 4
C 66300 44100 1 90 0 resistor-1.sym
{
T 65900 44400 5 10 0 0 90 0 1
device=RESISTOR
T 66000 44300 5 10 1 1 90 0 1
refdes=R8
T 66500 44300 5 10 1 1 90 0 1
value=330
T 66300 44100 5 10 0 0 90 0 1
footprint=0805
}
C 66400 42600 1 90 0 led-3.sym
{
T 66650 42350 5 10 1 1 90 0 1
device=YELLOW (RSSI) LED
T 65850 43050 5 10 1 1 90 0 1
refdes=D2
T 66400 42600 5 10 0 0 0 0 1
footprint=1206
}
N 66200 44100 66200 43500 4
N 66200 45200 66200 45000 4
C 72100 44100 1 90 0 resistor-1.sym
{
T 71700 44400 5 10 0 0 90 0 1
device=RESISTOR
T 71800 44300 5 10 1 1 90 0 1
refdes=R9
T 72300 44300 5 10 1 1 90 0 1
value=330
T 72100 44100 5 10 0 0 90 0 1
footprint=0805
}
C 72200 42600 1 90 0 led-3.sym
{
T 72450 42350 5 10 1 1 90 0 1
device=RED (ASSOC) LED
T 71650 43050 5 10 1 1 90 0 1
refdes=D3
T 72200 42600 5 10 0 0 0 0 1
footprint=1206
}
N 72000 44100 72000 43500 4
N 72000 45200 72000 45000 4
C 69600 50200 1 90 0 resistor-1.sym
{
T 69200 50500 5 10 0 0 90 0 1
device=RESISTOR
T 69300 50400 5 10 1 1 90 0 1
refdes=R7
T 69800 50400 5 10 1 1 90 0 1
value=1Meg
T 69600 50200 5 10 0 0 90 0 1
footprint=0805
T 69600 50100 5 10 1 1 0 0 1
description=1%
}
C 70000 54700 1 0 0 resistor-1.sym
{
T 70300 55100 5 10 0 0 0 0 1
device=RESISTOR
T 70200 55000 5 10 1 1 0 0 1
refdes=R6
T 70200 54500 5 10 1 1 0 0 1
value=4.99Meg
T 70000 54700 5 10 0 0 0 0 1
footprint=0805
T 70700 54900 5 10 1 1 0 0 1
description=1%
}
N 69500 51100 69500 55500 4
N 70900 54800 72000 54800 4
N 72000 54800 72000 61200 4
N 72000 61200 44400 61200 4
C 69400 49700 1 0 0 gnd-1.sym
N 69500 50000 69500 50200 4
N 71500 50000 71500 50300 4
C 44600 57300 1 270 0 capacitor-1.sym
{
T 45300 57100 5 10 0 1 270 0 1
device=CAPACITOR
T 44900 57000 5 10 1 1 0 0 1
refdes=C1
T 45500 57100 5 10 0 0 270 0 1
symversion=0.1
T 44900 56500 5 10 1 1 0 0 1
value=10uF
T 44900 56100 5 10 0 1 0 0 1
footprint=0805
T 44900 56300 5 10 1 1 0 0 1
comment=25V
}
N 43100 55400 57000 55400 4
N 44800 56400 44800 55400 4
N 44800 57300 44800 59800 4
N 42200 59800 43000 59800 4
{
T 42400 59800 5 10 1 1 0 0 1
netname=VIN
}
C 55600 57600 1 90 0 resistor-1.sym
{
T 55200 57900 5 10 0 0 90 0 1
device=RESISTOR
T 55300 57800 5 10 1 1 90 0 1
refdes=R4
T 55800 57800 5 10 1 1 90 0 1
value=330
T 55600 57600 5 10 0 0 90 0 1
footprint=0805
}
N 55500 58500 55500 59200 4
C 55700 56500 1 90 0 led-3.sym
{
T 55950 56450 5 10 1 1 90 0 1
device=GREEN LED
T 55150 56950 5 10 1 1 90 0 1
refdes=D1
T 55700 56500 5 10 0 0 0 0 1
footprint=1206
}
N 55500 57600 55500 57400 4
N 55500 45500 55500 56500 4
N 44400 61200 44400 59800 4
N 72000 42600 72000 41900 4
N 52500 41900 72000 41900 4
N 66200 42600 66200 41900 4
C 51900 41600 1 270 0 header2-1.sym
{
T 52550 40600 5 10 0 0 270 0 1
device=HEADER2
T 52800 41200 5 10 1 1 270 0 1
refdes=JP1
T 51900 41600 5 10 0 0 0 0 1
footprint=JUMPER2
}
N 52500 41600 52500 45500 4
C 51100 41300 1 0 1 gnd-1.sym
N 52100 41600 52100 41900 4
N 52100 41900 51000 41900 4
C 61000 56700 1 0 0 avrprog-1.sym
{
T 61000 58300 5 10 0 1 0 0 1
device=AVRPROG
T 61600 58000 5 10 1 1 0 0 1
refdes=J2
T 61200 56500 5 10 0 1 0 0 1
footprint=JUMPER3x2
}
N 70200 45500 69800 45500 4
C 64800 45800 1 270 0 capacitor-1.sym
{
T 65500 45600 5 10 0 1 270 0 1
device=CAPACITOR
T 65100 45500 5 10 1 1 0 0 1
refdes=C11
T 65700 45600 5 10 0 0 270 0 1
symversion=0.1
T 65100 45000 5 10 1 1 0 0 1
value=8.2pF
T 64800 45800 5 10 0 0 0 0 1
footprint=0805
T 65100 44800 5 10 1 1 0 0 1
description=16V, NP0
}
C 64900 44600 1 0 0 gnd-1.sym
N 65000 45800 65000 46700 4
C 57800 43100 1 0 1 rs485-1.sym
{
T 56150 44900 5 10 0 0 0 6 1
device=MAX489
T 56450 43250 5 10 1 1 0 6 1
refdes=XU6
T 56150 44700 5 10 0 0 0 6 1
footprint=DIP8
}
N 57800 44400 61200 44400 4
N 57800 44100 61500 44100 4
N 51000 42400 58700 42400 4
N 58000 42400 58000 43500 4
N 58000 43500 57800 43500 4
C 59300 45600 1 90 0 resistor-1.sym
{
T 58900 45900 5 10 0 0 90 0 1
device=RESISTOR
T 59000 45800 5 10 1 1 90 0 1
refdes=R12
T 59500 45800 5 10 1 1 90 0 1
value=330
T 59300 45600 5 10 0 0 90 0 1
footprint=0805
}
C 59400 44600 1 90 0 led-3.sym
{
T 59650 44550 5 10 1 1 90 0 1
device=AMBER LED
T 58850 45050 5 10 1 1 90 0 1
refdes=D4
T 59400 44600 5 10 0 0 0 0 1
footprint=1206
}
N 59200 45600 59200 45500 4
C 58300 44600 1 90 0 resistor-1.sym
{
T 57900 44900 5 10 0 0 90 0 1
device=RESISTOR
T 58000 44800 5 10 1 1 90 0 1
refdes=R10
T 58500 44800 5 10 1 1 90 0 1
value=10k
T 58300 44600 5 10 0 0 90 0 1
footprint=0805
}
C 58800 42600 1 90 0 resistor-1.sym
{
T 58400 42900 5 10 0 0 90 0 1
device=RESISTOR
T 58500 42800 5 10 1 1 90 0 1
refdes=R11
T 59000 42800 5 10 1 1 90 0 1
value=10k
T 58800 42600 5 10 0 0 90 0 1
footprint=0805
}
N 58200 44600 58200 44400 4
N 57000 43100 57000 42400 4
N 57000 45000 57000 46700 4
N 58200 46700 58200 45500 4
N 59200 44600 59200 43800 4
N 57800 43800 60900 43800 4
N 58700 43500 58700 44100 4
N 58700 42400 58700 42600 4
N 59200 46500 59200 46700 4
C 62300 45800 1 270 0 capacitor-1.sym
{
T 63000 45600 5 10 0 1 270 0 1
device=CAPACITOR
T 62600 45500 5 10 1 1 0 0 1
refdes=C12
T 63200 45600 5 10 0 0 270 0 1
symversion=0.1
T 62600 45000 5 10 1 1 0 0 1
value=1uF
T 62300 45800 5 10 0 0 0 0 1
footprint=0805
T 62600 44800 5 10 1 1 0 0 1
description=16V
}
N 62500 45800 62500 46700 4
C 62400 44600 1 0 0 gnd-1.sym
N 67100 48600 62100 48600 4
N 62100 48600 62100 52500 4
N 62100 52500 64200 52500 4
C 55500 43900 1 0 1 termblk2-1.sym
{
T 54500 44550 5 10 0 0 0 6 1
device=TERMBLK2
T 55100 43700 5 10 1 1 0 6 1
refdes=J4
T 55500 43900 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
T 54600 44400 9 10 1 0 0 6 1
RS485-A
T 54600 44000 9 10 1 0 0 6 1
RS485-B
N 55500 44100 55700 44100 4
N 55700 44100 55700 43700 4
N 55700 43700 56200 43700 4
N 55500 44500 56200 44500 4
N 55500 45500 52500 45500 4
N 51000 41600 51000 42400 4
N 56400 60400 56000 60400 4
N 56000 60400 56000 61200 4
C 57100 59500 1 0 0 gnd-1.sym
N 58000 60400 59000 60400 4
N 59000 60400 59000 59200 4
T 41000 41800 9 10 1 0 0 2 5
Notes:
1) All capacitors are ceramic (X7R/X5R) unless otherwise noted.
2) All capacitors and resistors are 0805 unless otherwise noted.
3) When programming MRBW version, do NOT supply
    5V power through the programming header with an XBee installed!
C 56800 57700 1 270 0 capacitor-1.sym
{
T 57500 57500 5 10 0 1 270 0 1
device=CAPACITOR
T 57100 57400 5 10 1 1 0 0 1
refdes=C3
T 57700 57500 5 10 0 0 270 0 1
symversion=0.1
T 57100 56900 5 10 1 1 0 0 1
value=10uF*
T 57100 56500 5 10 1 1 0 0 1
footprint=0805
T 57100 56700 5 10 1 1 0 0 1
comment=16V
}
N 57000 56800 57000 55400 4
N 57000 57700 57000 59200 4
B 45900 55700 7600 5200 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
B 55600 59400 3600 1600 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
T 53300 60400 9 10 1 0 0 6 2
MRBW
(3.3V)
T 55700 59500 9 10 1 0 0 0 2
MRB
(5V)
T 57000 55200 9 10 1 0 0 5 1
(* 1uF for MRB option)
L 53500 45000 53500 42200 3 0 0 0 -1 -1
L 53500 42200 60500 42200 3 0 0 0 -1 -1
L 60500 42200 60500 47000 3 0 0 0 -1 -1
L 60500 47000 56000 47000 3 0 0 0 -1 -1
L 56000 47000 56000 45000 3 0 0 0 -1 -1
L 53500 45000 56000 45000 3 0 0 0 -1 -1
T 56100 46700 9 10 1 0 0 0 1
MRB
B 64500 41700 8300 6000 3 0 0 0 -1 -1 0 -1 -1 -1 -1 -1
T 64600 47400 9 10 1 0 0 0 1
MRBW
C 43000 59600 1 0 0 schottky-1.sym
{
T 43322 60272 5 10 0 0 0 0 1
device=DIODE
T 43300 60100 5 10 1 1 0 0 1
refdes=D5
T 43341 60432 5 10 0 1 0 0 1
footprint=SOD123
T 42800 59300 5 10 1 1 0 0 1
model=MBR0520LT1G
}
N 43900 59800 44800 59800 4
N 43100 56400 43100 55400 4
N 43100 57300 43100 58000 4
N 42200 58000 44800 58000 4
C 42900 57300 1 270 0 Cap_H-2.sym
{
T 43200 57100 5 10 1 1 0 0 1
refdes=C1A
T 44400 57300 5 10 0 0 270 0 1
device=Capacitor
T 43200 56600 5 10 1 1 0 2 1
value=68uF
T 42900 57300 5 10 0 0 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
T 43200 56200 5 10 1 1 0 0 1
description=25V, Electrolytic
}
C 56400 59800 1 0 0 78l05-1.sym
{
T 58000 61100 5 10 0 0 0 0 1
device=7805
T 57800 60800 5 10 1 1 0 6 1
refdes=U1A
T 56400 59800 5 10 1 1 0 0 1
footprint=SOT89
}
N 62400 57700 62600 57700 4
N 62600 57700 62600 59200 4
N 67400 44300 67100 44300 4
N 67100 44300 67100 48600 4
C 63300 50300 1 90 1 capacitor-1.sym
{
T 62600 50100 5 10 0 1 270 2 1
device=CAPACITOR
T 63000 50000 5 10 1 1 0 6 1
refdes=C8
T 62400 50100 5 10 0 0 270 2 1
symversion=0.1
T 63000 49500 5 10 1 1 0 6 1
value=33pF
T 63300 50300 5 10 0 0 0 6 1
footprint=0805
T 63000 49300 5 10 1 1 0 6 1
description=16V, NP0
}
C 69700 56700 1 270 0 capacitor-1.sym
{
T 70400 56500 5 10 0 1 270 0 1
device=CAPACITOR
T 70000 56400 5 10 1 1 0 0 1
refdes=C6
T 70600 56500 5 10 0 0 270 0 1
symversion=0.1
T 70000 55900 5 10 1 1 0 0 1
value=0.1uF
T 69700 56700 5 10 0 0 0 0 1
footprint=0805
T 70000 55700 5 10 1 1 0 0 1
description=16V
}
C 70400 58800 1 90 1 capacitor-1.sym
{
T 69700 58600 5 10 0 1 270 2 1
device=CAPACITOR
T 70100 58500 5 10 1 1 0 6 1
refdes=C5
T 69500 58600 5 10 0 0 270 2 1
symversion=0.1
T 70100 58000 5 10 1 1 0 6 1
value=0.1uF
T 70400 58800 5 10 0 0 0 6 1
footprint=0805
T 70100 57800 5 10 1 1 0 6 1
description=16V
}
N 48600 56700 48600 55400 4
N 48000 55400 48000 56700 4
N 47300 57700 46900 57700 4
N 46900 57700 46900 59200 4
C 51200 58200 1 90 0 resistor-1.sym
{
T 50800 58500 5 10 0 0 90 0 1
device=RESISTOR
T 50900 58400 5 10 1 1 90 0 1
refdes=R1
T 51400 58400 5 10 1 1 90 0 1
value=499k
T 51200 58200 5 10 0 0 90 0 1
footprint=0805
T 51200 58100 5 10 1 1 0 0 1
description=1%
}
C 51200 56000 1 90 0 resistor-1.sym
{
T 50800 56300 5 10 0 0 90 0 1
device=RESISTOR
T 50900 56200 5 10 1 1 90 0 1
refdes=R2
T 51400 56200 5 10 1 1 90 0 1
value=287k
T 51200 56000 5 10 0 0 90 0 1
footprint=0805
T 51200 55900 5 10 1 1 0 0 1
description=1%
}
C 51900 58900 1 270 0 capacitor-1.sym
{
T 52600 58700 5 10 0 1 270 0 1
device=CAPACITOR
T 52200 58600 5 10 1 1 0 0 1
refdes=C2
T 52800 58700 5 10 0 0 270 0 1
symversion=0.1
T 52200 58100 5 10 1 1 0 0 1
value=33pF
T 52200 57900 5 10 0 1 0 0 1
footprint=0805
T 52200 57900 5 10 1 1 0 0 1
description=16V, NP0
}
N 51100 59100 51100 59200 4
N 51100 58200 51100 56900 4
N 51100 56000 51100 55400 4
N 49900 58100 51100 58100 4
N 52100 58900 52100 59200 4
N 52100 57700 52100 58000 4
C 47300 56700 1 0 0 ltc3528.sym
{
T 48600 58800 5 10 1 1 0 4 1
device=LTC3528
T 49600 59550 5 10 1 1 0 6 1
refdes=U1
T 49300 56600 5 10 1 1 0 0 1
footprint=LTC_DDB8
}
C 46700 60300 1 0 0 inductor-1.sym
{
T 46900 60800 5 10 0 0 0 0 1
device=INDUCTOR
T 46800 60600 5 10 1 1 0 0 1
refdes=L1
T 46900 61000 5 10 0 0 0 0 1
symversion=0.1
T 46700 60000 5 10 1 1 0 0 1
model=MSS6132-472
T 46700 60300 5 10 0 0 0 0 1
footprint=MSS6132
T 46700 60200 5 10 1 1 0 0 1
value=4.7uH
}
N 49200 56700 49200 55400 4
N 46700 60400 46300 60400 4
N 46300 60400 46300 59200 4
N 47600 60400 48600 60400 4
N 48600 60400 48600 59800 4
N 52100 57700 51100 57700 4
N 47300 59200 44800 59200 4
T 41300 57900 9 10 1 0 0 6 1
VBAT
N 69000 53400 69900 53400 4
{
T 70000 53400 5 10 1 1 0 1 1
netname=DATA
}
N 54200 52300 54200 59200 4
N 54200 53000 50400 53000 4
C 50600 43700 1 0 0 gnd-1.sym
N 50700 44000 50700 51800 4
N 50700 51800 50400 51800 4
N 51400 51500 52900 51500 4
{
T 53000 51500 5 10 1 1 0 1 1
netname=DATA
}
N 51400 51500 51400 52600 4
N 51400 52600 50400 52600 4
C 52500 51800 1 90 0 resistor-1.sym
{
T 52100 52100 5 10 0 0 90 0 1
device=RESISTOR
T 52200 52000 5 10 1 1 90 0 1
refdes=R3
T 52700 52000 5 10 1 1 90 0 1
value=1k
T 52500 51800 5 10 0 0 90 0 1
footprint=0805
}
N 52400 52700 52400 53000 4
N 52400 51800 52400 51500 4
C 54000 52300 1 270 0 capacitor-1.sym
{
T 54700 52100 5 10 0 1 270 0 1
device=CAPACITOR
T 54300 52000 5 10 1 1 0 0 1
refdes=C4
T 54900 52100 5 10 0 0 270 0 1
symversion=0.1
T 54300 51500 5 10 1 1 0 0 1
value=0.1uF
T 54000 52300 5 10 0 0 0 0 1
footprint=0805
T 54300 51300 5 10 1 1 0 0 1
description=16V
}
C 54100 51100 1 0 0 gnd-1.sym
C 50400 51600 1 0 1 rht03.sym
{
T 50100 51350 5 10 1 1 0 6 1
device=RHT03
T 49900 53300 5 10 1 1 0 6 1
refdes=U3
T 50400 51600 5 10 0 0 0 0 1
footprint=RHT03
}
N 48100 44500 48100 54000 4
N 48100 54000 54200 54000 4
N 45900 48600 53400 48600 4
{
T 53500 48600 5 10 1 1 0 1 1
netname=SDA
}
N 45900 47400 53400 47400 4
{
T 53500 47400 5 10 1 1 0 1 1
netname=SCL
}
N 69000 53100 69900 53100 4
{
T 70000 53100 5 10 1 1 0 1 1
netname=SDA
}
N 69000 52800 69900 52800 4
{
T 70000 52800 5 10 1 1 0 1 1
netname=SCL
}
C 52200 49000 1 90 0 resistor-1.sym
{
T 51800 49300 5 10 0 0 90 0 1
device=RESISTOR
T 51900 49200 5 10 1 1 90 0 1
refdes=R13
T 52400 49200 5 10 1 1 90 0 1
value=2k
T 52200 49000 5 10 0 0 90 0 1
footprint=0805
}
C 53100 49000 1 90 0 resistor-1.sym
{
T 52700 49300 5 10 0 0 90 0 1
device=RESISTOR
T 52800 49200 5 10 1 1 90 0 1
refdes=R14
T 53300 49200 5 10 1 1 90 0 1
value=2k
T 53100 49000 5 10 0 0 90 0 1
footprint=0805
}
N 52100 49000 52100 48600 4
N 53000 49000 53000 47400 4
N 53000 49900 53000 50300 4
N 53000 50300 48100 50300 4
N 52100 49900 52100 50300 4
N 70000 54800 69500 54800 4
N 69500 55500 69000 55500 4
C 42200 57800 1 0 1 header1-1.sym
{
T 41200 58450 5 10 0 0 0 6 1
device=HEADER2
T 41800 58300 5 10 1 1 0 6 1
refdes=J3
T 42200 57800 5 10 0 0 0 0 1
footprint=JUMPER1
}
C 48600 44000 1 0 0 tmp275.sym
{
T 50100 46050 5 10 1 1 0 6 1
refdes=U7
T 48900 46050 5 10 1 1 0 0 1
device=TMP275
T 49700 44000 5 10 1 1 0 0 1
footprint=SO8
}
N 46500 55100 46500 55400 4
C 46400 54800 1 0 0 gnd-1.sym
N 48100 44500 48600 44500 4
N 50400 45700 50700 45700 4
N 50400 44500 51600 44500 4
N 50400 44900 51200 44900 4
N 48600 44900 48100 44900 4
N 48600 45300 48100 45300 4
N 48600 45700 48100 45700 4
N 51600 44500 51600 48600 4
N 51200 44900 51200 47400 4
C 45700 49400 1 0 1 header4-1.sym
{
T 44700 50050 5 10 0 0 0 6 1
device=HEADER3
T 45300 51100 5 10 1 1 0 6 1
refdes=J5
T 45700 49400 5 10 0 0 0 0 1
footprint=JUMPER4-050
}
C 57700 51800 1 0 1 header6-1.sym
{
T 56700 52450 5 10 0 0 0 6 1
device=HEADER3
T 57300 54300 5 10 1 1 0 6 1
refdes=J6
T 57700 51800 5 10 0 0 0 0 1
footprint=JUMPER6-050
}
C 45900 47200 1 0 1 hyt221.sym
{
T 44900 47850 5 10 0 0 0 6 1
device=RHT03
T 45400 48900 5 10 1 1 0 6 1
refdes=U4
T 45000 47000 5 10 1 1 0 0 1
footprint=HYT221
}
N 45900 48200 50700 48200 4
N 45900 47800 50300 47800 4
N 50300 47800 50300 50300 4
N 45700 49600 46400 49600 4
N 46400 49600 46400 47400 4
N 46700 50000 45700 50000 4
N 45700 50400 47000 50400 4
N 47300 50800 45700 50800 4
N 46700 50000 46700 47800 4
N 47000 50400 47000 48200 4
N 47300 50800 47300 48600 4
N 69000 53700 69900 53700 4
{
T 70000 53700 5 10 1 1 0 1 1
netname=\_CS
}
N 57700 52400 58600 52400 4
{
T 58700 52400 5 10 1 1 0 1 1
netname=\_CS
}
C 57900 51500 1 0 0 gnd-1.sym
N 57700 52000 58000 52000 4
N 58000 52000 58000 51800 4
N 57700 54000 59600 54000 4
N 58500 56400 58500 53600 4
N 58500 53600 57700 53600 4
N 58900 52800 58900 56100 4
N 58900 52800 57700 52800 4
N 57700 53200 63400 53200 4
