
#include <inttypes.h>
#include "opendefs.h"
#include "sensorlab-frame-format.h"


#define OBSERVER_ON_DUTY TRUE

#define OBSERVER_CODE ((uint8_t)'O')

#define OBSERVER_BUFFER_LENGTH 512

#define OBSERVER_BUFFER_EMPTY 0x00
#define OBSERVER_BUFFER_OVERWRITE 0xFF
#define OBSERVER_BUFFER_OVERFLOW 0x0F


typedef struct observer_vars
{
	uint8_t buffer[OBSERVER_BUFFER_LENGTH];
	uint16_t cursor;
	uint8_t properties_left_count;
	bool overflow_error;
} observer_vars_t;

owerror_t observer_init(void);
owerror_t observer_submit(void);


void observer_node_add(uint8_t properties_count);
void observer_node_property_add(uint8_t properties_count);
void observer_node_property_update(uint8_t properties_count);
void observer_node_remove(void);

void observer_entity_add(uint8_t id, char* name, uint8_t properties_count);
void observer_entity_property_add(uint8_t id, uint8_t properties_count);
void observer_entity_property_update(uint8_t id, uint8_t properties_count);
void observer_entity_remove(uint8_t id);

void observer_link_add(uint8_t entity_id, uint8_t link_id, uint8_t src_properties_count, uint8_t tgt_properties_count, uint8_t properties_count);
void observer_link_property_add(uint8_t entity_id, uint8_t link_id, uint8_t properties_count);
void observer_link_property_update(uint8_t entity_id, uint8_t link_id, uint8_t properties_count);
void observer_link_remove(uint8_t entity_id, uint8_t link_id);

void observer_frame_produce(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data, uint8_t properties_count);
void observer_frame_property_add(uint8_t entity_id, uint8_t frame_id, uint8_t properties_count);
void observer_frame_property_update(uint8_t entity_id, uint8_t frame_id, uint8_t properties_count);
void observer_frame_data_update(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data);
void observer_frame_tx(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data);
void observer_frame_rx(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data, uint8_t properties_count);
void observer_frame_consume(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data);


void observer_property_declaration_boolean(uint8_t id, char* name, uint8_t value);
void observer_property_declaration_int8(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int8_t value);
void observer_property_declaration_uint8(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint8_t value);
void observer_property_declaration_int16(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int16_t value);
void observer_property_declaration_uint16(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint16_t value);
void observer_property_declaration_int32(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int32_t value);
void observer_property_declaration_uint32(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint32_t value);
void observer_property_declaration_int64(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int64_t value);
void observer_property_declaration_uint64(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint64_t value);
void observer_property_declaration_float(uint8_t id, char* name, uint8_t prefix, uint8_t unit, float value);
void observer_property_declaration_double(uint8_t id, char* name, uint8_t prefix, uint8_t unit, double value);
void observer_property_declaration_ASCII_array(uint8_t id, char* name, uint16_t value_length, char* value);
void observer_property_declaration_byte_array(uint8_t id, char* name, uint16_t value_length, uint8_t* value);


void observer_property_update_boolean(uint8_t id, uint8_t value);
void observer_property_update_int8(uint8_t id, int8_t value);
void observer_property_update_uint8(uint8_t id, uint8_t value);
void observer_property_update_int16(uint8_t id, int16_t value);
void observer_property_update_uint16(uint8_t id, uint16_t value);
void observer_property_update_int32(uint8_t id, int32_t value);
void observer_property_update_uint32(uint8_t id, uint32_t value);
void observer_property_update_int64(uint8_t id, int64_t value);
void observer_property_update_uint64(uint8_t id, uint64_t value);
void observer_property_update_float(uint8_t id, float value);
void observer_property_update_double(uint8_t id, double value);
void observer_property_update_ASCII_array(uint8_t id, uint16_t value_length, char* value);
void observer_property_update_byte_array(uint8_t id, uint16_t value_length, uint8_t* value);

void observer_property_reference_boolean(uint8_t id, uint8_t value);
void observer_property_reference_int8(uint8_t id, int8_t value);
void observer_property_reference_uint8(uint8_t id, uint8_t value);
void observer_property_reference_int16(uint8_t id, int16_t value);
void observer_property_reference_uint16(uint8_t id, uint16_t value);
void observer_property_reference_int32(uint8_t id, int32_t value);
void observer_property_reference_uint32(uint8_t id, uint32_t value);
void observer_property_reference_int64(uint8_t id, int64_t value);
void observer_property_reference_uint64(uint8_t id, uint64_t value);
void observer_property_reference_float(uint8_t id, float value);
void observer_property_reference_double(uint8_t id, double value);
void observer_property_reference_ASCII_array(uint8_t id, uint16_t value_length, char* value);
void observer_property_reference_byte_array(uint8_t id, uint16_t value_length, uint8_t* value);


void owsn_observer_frame_produce(OpenQueueEntry_t *packet, uint8_t properties_count);
void owsn_observer_frame_property_add(OpenQueueEntry_t *packet, uint8_t properties_count);
void owsn_observer_frame_property_update(OpenQueueEntry_t *packet, uint8_t properties_count);
void owsn_observer_frame_data_update(OpenQueueEntry_t *packet);
void owsn_observer_frame_tx(OpenQueueEntry_t *packet);
void owsn_observer_frame_rx(OpenQueueEntry_t *packet, uint8_t properties_count);
void owsn_observer_frame_consume(OpenQueueEntry_t *packet);

void owsn_observer_link_l1_add(neighborRow_t * neighbor);
void owsn_observer_link_l2_add(neighborRow_t * neighbor);
void owsn_observer_link_l3_add(uint8_t id, open_addr_t * address, dagrank_t rank);
void owsn_observer_link_l1_remove(neighborRow_t * neighbor);
void owsn_observer_link_l2_remove(neighborRow_t * neighbor);
void owsn_observer_link_l3_remove(uint8_t id);
