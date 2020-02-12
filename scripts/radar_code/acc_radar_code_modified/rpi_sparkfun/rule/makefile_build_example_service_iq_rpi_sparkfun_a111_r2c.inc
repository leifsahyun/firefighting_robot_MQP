BUILD_ALL += $(OUT_DIR)/example_service_iq_rpi_sparkfun_a111_r2c

$(OUT_DIR)/example_service_iq_rpi_sparkfun_a111_r2c : \
					$(OUT_OBJ_DIR)/example_service_iq.o \
					libacconeer.a \
					libcustomer.a \
					$(OUT_OBJ_DIR)/acc_board_rpi_sparkfun.o
	@echo "    Linking $(notdir $@)"
	$(SUPPRESS)$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LDLIBS) -o $@
