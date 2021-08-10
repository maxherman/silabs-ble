#!/bin/sh

# Programming tool
COMMANDER="~/Tools/SimplicityStudio-5/SimplicityStudio_v5/developer/adapter_packs/commander/commander"

# Image to program (hex will not overwrite the bootloader!)
IMG_HEX="/home/jeffrey/git/Gen4Ble/soc_aoa_asset_tag/GNU ARM v7.2.1 - Debug/Gen4Ble.hex"

#echo "Hello!"
# Program all 26 boards
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440171812"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440180753"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440179021"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440180722"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440180762"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440172415"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440171816"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440166683"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440166681"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440167608"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440168187"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440166518"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440167571"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440167513"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440166525"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440171613"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440166694"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440166591"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440170862"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440167534"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440167478"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440167620"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440167257"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440171780"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440171951"
bash -c "$COMMANDER flash '$IMG_HEX' --serialno 440166693"
