BUILD_ALL += $(OUT_DIR)/service_envelope_modified

$(OUT_DIR)/service_envelope_modified : \
					$(OUT_OBJ_DIR)/service_envelope_modified.o \
					libacconeer.a \
					libcustomer.a \
					$(OUT_OBJ_DIR)/acc_board_rpi_sparkfun.o
	@echo "    Linking $(notdir $@)"
	$(SUPPRESS)$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LDLIBS) -o $@
