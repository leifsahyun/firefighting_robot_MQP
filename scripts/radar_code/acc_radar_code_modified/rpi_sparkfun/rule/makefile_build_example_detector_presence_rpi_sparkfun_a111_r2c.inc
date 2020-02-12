BUILD_ALL += $(OUT_DIR)/example_detector_presence_rpi_sparkfun_a111_r2c

$(OUT_DIR)/example_detector_presence_rpi_sparkfun_a111_r2c : \
					$(OUT_OBJ_DIR)/example_detector_presence.o \
					libacc_detector_presence.a \
					libacconeer.a \
					libcustomer.a \
					$(OUT_OBJ_DIR)/acc_board_rpi_sparkfun.o
	@echo "    Linking $(notdir $@)"
	$(SUPPRESS)$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LDLIBS) -o $@


BUILD_ALL += $(OUT_DIR)/ref_app_smart_presence_rpi_sparkfun_a111_r2c

$(OUT_DIR)/ref_app_smart_presence_rpi_sparkfun_a111_r2c : \
					$(OUT_OBJ_DIR)/ref_app_smart_presence.o \
					libacc_detector_presence.a \
					libacconeer.a \
					libcustomer.a \
					$(OUT_OBJ_DIR)/acc_board_rpi_sparkfun.o
	@echo "    Linking $(notdir $@)"
	$(SUPPRESS)$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LDLIBS) -o $@
