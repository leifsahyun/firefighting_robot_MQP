#include <stdio.h>

// Author: Leif Sahyun
// Based on Acconeer Envelope Service User Guide

int main(int argc, char *argv[]) {
  /*** SETUP AND INITIALIZATION ***/
  if (! acc_driver_hal_init() )
  {
    /* Handle error */
  }
  acc_hal_t hal = acc_driver_hal_get_implementation();
  if (! acc_rss_activate(&hal) )
  {
    /* Handle error */
  }

  acc_service_configuration_t envelope_configuration = acc_service_envelope_configuration_create();
  if ( envelope_configuration == NULL )
  {
    /* Handle error */
  }
  
  //set the service profile for the envelope_configuration to high gain
  acc_service_profile_set(envelope_configuration, ACC_SERVICE_PROFILE_4);
  
  //create the service handle from the configuration
  acc_service_handle_t handle = acc_service_create(envelope_configuration);
  if ( handle == NULL )
  {
    /* Handle error */
  }
  
  //determine how much space we need to allocate for each reading
  acc_service_envelope_metadata_t envelope_metadata;
  acc_service_envelope_get_metadata(handle, &envelope_metadata);
  int data_size = envelope_metadata.data_length;
  uint16_t data [data_size];
  acc_service_envelope_result_info_t result_info;
  /*** setup complete ***/

  /*** ACTUALLY READING FROM SENSOR ***/
  //loop for continuous readings
  for(int i = 0; i < 10; i++) {
    /*** THIS FUNCTION GETS AN ENVELOPE OF READINGS ***/
    bool read_success = acc_service_envelope_get_next(handle, data, envelope_metadata.data_length, &result_info);
    /*** READING IS STORED IN ARRAY DATA ***/
    //print the data
    for(int j = 0; j < data_size; j++) {
      printf("%d, ", data[j]);
    }
    if(!read_success)
    {
      /* Handle Error */
    }
  }

  /*** SHUTDOWN THE SENSOR - CLEAN UP ***/
  acc_service_deactivate(handle);
  acc_service_destroy(&handle);
  acc_rss_deactivate();

}
