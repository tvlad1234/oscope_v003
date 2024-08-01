all : flash

TARGET:=oscope_v003

ADDITIONAL_C_FILES+=ssd1306.c gfx.c 

CH32V003FUN:=../ch32v003fun/ch32v003fun

include $(CH32V003FUN)/ch32v003fun.mk

flash : cv_flash
clean : cv_clean