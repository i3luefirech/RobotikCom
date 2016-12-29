# RobotikCom

RobotikCom ist eine Embedded Software für das STM32F4Discovery Board.

Mit einem Buildswitch (#define) kann die SW als Antriebs/Antriebsdecoder simulator gebaut werden, oder als "MainController".

Das Ziel dieses Projekt ist es, das Kommunikationsprinzip eines Roboters mit CAN darzustellen.

## Antrieb/Antriebsdecoder

Die Software simuliert auf ganz einfache Art einen Antrieb und einen Antriebsdecoder.
Die veränderten Werte werden über CAN übertragen.

## MainController

Die Software simuliert den MainController eines Roboters.
Er monitort alle versendeten Pakete und sendet Befehle an den Antieb/Antriebsdecoder

## Visualisierung

Zur Visualisierung der CAN Kommunikation wird entwerder auf einen PCAN-USB Dongle oder auf einen Digitalen Logic Analyzer zurück gegriffen!
