#
# Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
#
# This software, including source code, documentation and related
# materials ("Software") is owned by Cypress Semiconductor Corporation
# or one of its affiliates ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products.  Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#

ifneq ($(filter build program clean all,$(MAKECMDGOALS)),)

CY_DS2_INCLUDES+=\
    $(CY_BASELIB_PATH)/include \
    $(CY_BASELIB_PATH)/include/hal \
    $(CY_BASELIB_PATH)/include/internal \
    $(CY_BASELIB_PATH)/include/stack \
    $(CY_BASELIB_PATH)/WICED/common \
    $(CY_BASELIB_PATH)/internal/$(CY_TARGET_DEVICE) \
    $(CY_BSP_PATH) \
    $(SEARCH_btsdk-include)

CY_DS2_OBJCOPY=$(CY_CROSSPATH)/arm-none-eabi-objcopy
CY_DS2_OBJDUMP=$(CY_CROSSPATH)/arm-none-eabi-objdump
CY_DS2_HEX2BIN=$(CY_WICED_TOOLS_DIR)/IntelHexToBin/IntelHexToBin

DS2_APP_LIBS+=$(CY_BASELIB_PATH)/libraries/prebuilt/ofu_ds2_lib.a

# for 20189 with XIP, we use a DS2 app to copy OTA upgrade image to DS1
DS2_APP_ELF=$(CY_DS2_APP_HEX:.hex=.elf)
DS2_APP_SRC_FILES=ds2_app_ofu_copy.c
DS2_APP_SRC=$(addprefix $(DS2_APP_PATH)/,$(DS2_APP_SRC_FILES))
DS2_BUILD_PATH=$(dir $(CY_DS2_APP_HEX))
DS2_APP_OBJ=$(addprefix $(DS2_BUILD_PATH),$(DS2_APP_SRC_FILES:.c=.o))

CY_DS2_POSTBUILD= \
  $(CY_INTERNAL_TOOL_arm-none-eabi-objcopy_EXE) -j .app_xip_area -O ihex $(DS2_APP_ELF) $(DS2_APP_ELF:.elf=.hex); \
  chmod 0644 $(DS2_APP_ELF:.elf=.hex); \
  $(CY_DS2_HEX2BIN) -f 00 $(DS2_APP_ELF:.elf=.hex) $(DS2_APP_ELF:.elf=.bin); \
  $(CY_DS2_OBJDUMP) --disassemble $(DS2_APP_ELF) > $(DS2_APP_ELF:.elf=.asm);

CY_DS2_LDFLAGS=\
    $(LDFLAGS)\
    $(CY_TOOLCHAIN_LDFLAGS)\
    $(CY_CORE_LDFLAGS)\
    -Wl,--entry=ds2_app_entry \
    -Wl,--just-symbols="$(CY_CORE_PATCH_SYMBOLS)"\
    -T$(DS2_APP_PATH)/ds2_app.ld

$(DS2_APP_OBJ): $(DS2_APP_SRC)
	@echo "compiling ds2 app"
	mkdir -p $(@D)
	$(CY_NOISE)$(CC) $(CY_BUILD_ALL_CFLAGS) $(addprefix -I,$(CY_DS2_INCLUDES)) $(CY_APP_OTA_DEFINES) $(addprefix -D,$(CY_DS2_DEFINES)) \
		$(CY_TOOLCHAIN_DEPENDENCIES) $(CY_TOOLCHAIN_OUTPUT_OPTION) $@ $^

$(DS2_APP_ELF): $(DS2_APP_OBJ)
	@echo "linking ds2 app"
	$(CY_NOISE)$(LD) $(CY_DS2_LDFLAGS) $(CY_TOOLCHAIN_OUTPUT_OPTION) $@ $^ $(DS2_APP_LIBS)

$(CY_DS2_APP_HEX): $(DS2_APP_ELF)
	@echo "post build ds2 app"
	$(CY_NOISE)$(CY_DS2_POSTBUILD)

CY_BUILD_postbuild: $(CY_DS2_APP_HEX)

clean_ds2_hex:
	@echo "cleaning ds2 app"
	$(CY_NOISE)-rm -rf $(DS2_APP_PATH)/build/$(TARGET)

clean: clean_ds2_hex

endif
