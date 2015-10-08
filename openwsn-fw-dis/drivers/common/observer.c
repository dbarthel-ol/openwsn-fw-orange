

#include "opendefs.h"
#include "neighbors.h"
#include "idmanager.h"
#include "openserial.h"
#include "observer.h"
#include "sensorlab-frame-format.h"
#include "leds.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#ifdef SENSORLAB

static observer_vars_t observer_vars = {.cursor=1}; // forcing cursor value to 1 to avoid logging prior to observer_init call.

owerror_t observer_init(void){
#if OBSERVER_ON_DUTY
   memset(observer_vars.buffer, 0, OBSERVER_BUFFER_LENGTH * sizeof(uint8_t));
   observer_vars.cursor = 0;
   observer_vars.properties_left_count = 0;
   observer_vars.overflow_error = FALSE;
   return E_SUCCESS;
#else
   return E_SUCCESS;
#endif
}

owerror_t observer_new_event(void){
#if OBSERVER_ON_DUTY
   if (observer_vars.cursor != 0 || observer_vars.properties_left_count != 0){
      return E_FAIL;
   } else {
      INTERRUPT_DECLARATION();
      DISABLE_INTERRUPTS();
      memset(observer_vars.buffer, 0, OBSERVER_BUFFER_LENGTH * sizeof(uint8_t));
      observer_vars.buffer[observer_vars.cursor++] = OBSERVER_CODE;
      return E_SUCCESS;
   }
#else
   return E_SUCCESS;
#endif
}

owerror_t observer_submit(void){
#if OBSERVER_ON_DUTY
   if(observer_vars.cursor == 0){
      return E_FAIL;
   }else{
      if(!observer_vars.overflow_error){
         openserial_write(observer_vars.buffer, observer_vars.cursor);
         observer_vars.cursor = 0;
         observer_vars.properties_left_count = 0;
         INTERRUPT_DECLARATION();
         ENABLE_INTERRUPTS();
         return E_SUCCESS;
      }else{
         observer_vars.buffer[0] = OBSERVER_BUFFER_OVERFLOW;
         openserial_write(observer_vars.buffer, 1);
         observer_vars.cursor = 0;
         observer_vars.properties_left_count = 0;
         observer_vars.overflow_error = FALSE;
         INTERRUPT_DECLARATION();
         ENABLE_INTERRUPTS();
         return E_FAIL;
      }
   }
#else
   return E_SUCCESS;
#endif
}




void observer_node_add(uint8_t properties_count){
   /**
   | `propertiesCount` | `properties [...]` |
   |:-----------------:|:------------------:|
   |        8bits      |   variable size    |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_NODE_ADD;
      observer_vars.buffer[observer_vars.cursor++] = properties_count;
      observer_vars.properties_left_count = properties_count;
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_node_property_add(uint8_t properties_count){
   /**
   | `propertiesCount` | `properties [...]` |
   |:-----------------:|:------------------:|
   |        8bits      |   variable size    |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_NODE_PROPERTY_ADD;
      observer_vars.buffer[observer_vars.cursor++] = properties_count;
      observer_vars.properties_left_count = properties_count;
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_node_property_update(uint8_t properties_count){
   /**
   | `propertiesCount` | `properties [...]` |
   |:-----------------:|:------------------:|
   |        8bits      |   variable size    |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_NODE_PROPERTY_UPDATE;
      observer_vars.buffer[observer_vars.cursor++] = properties_count;
      observer_vars.properties_left_count = properties_count;
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_node_remove(void){
   /**
   | empty |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_NODE_REMOVE;
      observer_submit();
   }
}

void observer_entity_add(uint8_t id, char* name, uint8_t properties_count){
   uint8_t name_length;
   /**
   | `entityID` | `entityNameLength` | `propertiesCount` |   `entityName`  | `properties [...]` |
   |:----------:|:------------------:|:-----------------:|:---------------:|:------------------:|
   |    8bits   |      8bits         |    8bits          | variable size   |  variable size     |
   **/
   name_length = strlen(name);
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_ENTITY_ADD;
      observer_vars.buffer[observer_vars.cursor++] = id;
      observer_vars.buffer[observer_vars.cursor++] = name_length;
      observer_vars.buffer[observer_vars.cursor++] = properties_count;
      memcpy(&(observer_vars.buffer[observer_vars.cursor]), name, name_length);
      observer_vars.cursor += name_length;
      observer_vars.properties_left_count = properties_count;
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_entity_property_add(uint8_t id, uint8_t properties_count){
   /**
   | `entityID` | `entityNameLength` | `propertiesCount` |   `entityName`  | `properties [...]` |
   |:----------:|:------------------:|:-----------------:|:---------------:|:------------------:|
   |    8 bits  |      8 bits        |    8 bits         | variable size   |  variable size     |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_ENTITY_PROPERTY_ADD;
      observer_vars.buffer[observer_vars.cursor++] = id;
      observer_vars.buffer[observer_vars.cursor++] = properties_count;
      observer_vars.properties_left_count = properties_count;
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_entity_property_update(uint8_t id, uint8_t properties_count){
   /**
   | `entityID` | `propertiesCount` | `properties [...]` |
   |:----------:|:-----------------:|:------------------:|
   |    8 bits  |       8 bits      |   variable size    |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_ENTITY_PROPERTY_UPDATE;
      observer_vars.buffer[observer_vars.cursor++] = id;
      observer_vars.buffer[observer_vars.cursor++] = properties_count;
      observer_vars.properties_left_count = properties_count;
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_entity_remove(uint8_t id){
   /**
   | `entityID` |
   |:----------:|
   |    8 bits  |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_ENTITY_REMOVE;
      observer_vars.buffer[observer_vars.cursor++] = id;
      observer_submit();
   }
}

void observer_link_add(uint8_t entity_id, uint8_t link_id, uint8_t src_properties_count, uint8_t tgt_properties_count, uint8_t properties_count){
   /**
   | `entityID` | `linkID` | `sourcePropertiesCount` | `targetPropertiesCount` | `linkPropertiesCount` | `sourceProperties [...]` | `targetProperties [...]` | `linkProperties [...]` |
   |:----------:|:--------:|:-----------------------:|:-----------------------:|:---------------------:|:------------------------:|:------------------------:|:----------------------:|
   |   8 bits   |  8 bits  |          8 bits         |           8 bits        |          8 bits       |       variable size      |      variable size       |      variable size     |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_LINK_ADD;
      observer_vars.buffer[observer_vars.cursor++] = entity_id;
      observer_vars.buffer[observer_vars.cursor++] = link_id;
      observer_vars.buffer[observer_vars.cursor++] = src_properties_count;
      observer_vars.buffer[observer_vars.cursor++] = tgt_properties_count;
      observer_vars.buffer[observer_vars.cursor++] = properties_count;
      observer_vars.properties_left_count = src_properties_count + tgt_properties_count + properties_count;
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_link_property_add(uint8_t entity_id, uint8_t link_id, uint8_t properties_count){
   /**
   | `entityID` | `linkID` | `linkPropertiesCount` | `linkProperties [...]` |
   |:----------:|:--------:|:---------------------:|:----------------------:|
   |    8 bits  |   8 bits |         8 bits        |      variable size     |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_LINK_PROPERTY_ADD;
      observer_vars.buffer[observer_vars.cursor++] = entity_id;
      observer_vars.buffer[observer_vars.cursor++] = link_id;
      observer_vars.buffer[observer_vars.cursor++] = properties_count;
      observer_vars.properties_left_count = properties_count;
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_link_property_update(uint8_t entity_id, uint8_t link_id, uint8_t properties_count){
   /**
   | `entityID` | `linkID` | `linkPropertiesCount` | `linkProperties [...]` |
   |:----------:|:--------:|:---------------------:|:----------------------:|
   |    8 bits  |   8 bits |         8 bits        |      variable size     |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_LINK_PROPERTY_UPDATE;
      observer_vars.buffer[observer_vars.cursor++] = entity_id;
      observer_vars.buffer[observer_vars.cursor++] = link_id;
      observer_vars.buffer[observer_vars.cursor++] = properties_count;
      observer_vars.properties_left_count = properties_count;
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_link_remove(uint8_t entity_id, uint8_t link_id){
   /**
   | `entityID` | `linkID` |
   |:----------:|:--------:|
   |    8 bits  |   8 bits |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_LINK_PROPERTY_UPDATE;
      observer_vars.buffer[observer_vars.cursor++] = entity_id;
      observer_submit();
   }
}

void observer_frame_produce(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data, uint8_t properties_count){
   /**
   | `entityID` | `frameID` |  `dataLength` | `propertiesCount` |     `data`      |  `frameProperties [...]` |
   |:----------:|:---------:|:-------------:|:-----------------:|:---------------:|:------------------------:|
   |    8 bits  |   8 bits  |    16 bits    |        8 bits     |  variable size  |      variable size       |
   **/
   if( observer_new_event() == E_SUCCESS ){
      if(5 + data_length < OBSERVER_BUFFER_LENGTH){
         observer_vars.buffer[observer_vars.cursor++] = EVENT_FRAME_PRODUCE;
         observer_vars.buffer[observer_vars.cursor++] = entity_id;
         observer_vars.buffer[observer_vars.cursor++] = frame_id;
         observer_vars.buffer[observer_vars.cursor++] = data_length >> 0 & 0xff;
         observer_vars.buffer[observer_vars.cursor++] = data_length >> 8 & 0xff;
         observer_vars.buffer[observer_vars.cursor++] = properties_count;
         memcpy(&(observer_vars.buffer[observer_vars.cursor]), data, data_length);
         observer_vars.cursor += data_length;
         observer_vars.properties_left_count = properties_count;
      }else{
         observer_vars.overflow_error = TRUE;
      }
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_frame_property_add(uint8_t entity_id, uint8_t frame_id, uint8_t properties_count){
   /**
   | `entityID` | `frameID` | `propertiesCount` |  `frameProperties [...]` |
   |:----------:|:---------:|:-----------------:|:------------------------:|
   |    8 bits  |   8 bits  |        8 bits     |      variable size       |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_FRAME_PROPERTY_ADD;
      observer_vars.buffer[observer_vars.cursor++] = entity_id;
      observer_vars.buffer[observer_vars.cursor++] = frame_id;
      observer_vars.buffer[observer_vars.cursor++] = properties_count;
      observer_vars.properties_left_count = properties_count;
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_frame_property_update(uint8_t entity_id, uint8_t frame_id, uint8_t properties_count){
   /**
   | `entityID` | `frameID` | `propertiesCount` |  `frameProperties [...]` |
   |:----------:|:---------:|:-----------------:|:------------------------:|
   |    8 bits  |   8 bits  |        8 bits     |      variable size       |
   **/
   if( observer_new_event() == E_SUCCESS ){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_FRAME_PROPERTY_UPDATE;
      observer_vars.buffer[observer_vars.cursor++] = entity_id;
      observer_vars.buffer[observer_vars.cursor++] = frame_id;
      observer_vars.buffer[observer_vars.cursor++] = properties_count;
      observer_vars.properties_left_count = properties_count;
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_frame_data_update(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data){
   /**
   | `entityID` | `frameID` |  `dataLength` |     `data`      |
   |:----------:|:---------:|:-------------:|:---------------:|
   |    8 bits  |   8 bits  |    16 bits    |  variable size  |
   **/
   if( observer_new_event() == E_SUCCESS ){
      if(4 + data_length < OBSERVER_BUFFER_LENGTH){
         observer_vars.buffer[observer_vars.cursor++] = EVENT_FRAME_DATA_UPDATE;
         observer_vars.buffer[observer_vars.cursor++] = entity_id;
         observer_vars.buffer[observer_vars.cursor++] = frame_id;
         observer_vars.buffer[observer_vars.cursor++] = data_length >> 0 & 0xff;
         observer_vars.buffer[observer_vars.cursor++] = data_length >> 8 & 0xff;
         memcpy(&(observer_vars.buffer[observer_vars.cursor]), data, data_length);
         observer_vars.cursor += data_length;
      }else{
         observer_vars.overflow_error = TRUE;
      }
      observer_submit();
   }
}

void observer_frame_tx(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data){
   /**
   | `entityID` | `frameID` |  `dataLength` |     `data`      |
   |:----------:|:---------:|:-------------:|:---------------:|
   |    8 bits  |   8 bits  |    16 bits    |  variable size  |
   **/
   if( observer_new_event() == E_SUCCESS ){
      if(4 + data_length < OBSERVER_BUFFER_LENGTH){
         observer_vars.buffer[observer_vars.cursor++] = EVENT_FRAME_TX;
         observer_vars.buffer[observer_vars.cursor++] = entity_id;
         observer_vars.buffer[observer_vars.cursor++] = frame_id;
         observer_vars.buffer[observer_vars.cursor++] = data_length >> 0 & 0xff;
         observer_vars.buffer[observer_vars.cursor++] = data_length >> 8 & 0xff;
         memcpy(&(observer_vars.buffer[observer_vars.cursor]), data, data_length);
         observer_vars.cursor += data_length;
      }else{
         observer_vars.overflow_error = TRUE;
      }
      observer_submit();
   }
}


void observer_frame_rx(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data, uint8_t properties_count){
   /**
   | `entityID` | `frameID` |  `dataLength` | `propertiesCount` |     `data`      |  `frameProperties [...]` |
   |:----------:|:---------:|:-------------:|:-----------------:|:---------------:|:------------------------:|
   |    8 bits  |   8 bits  |    16 bits    |        8 bits     |  variable size  |      variable size       |
   **/
   if( observer_new_event() == E_SUCCESS ){
      if(5 + data_length < OBSERVER_BUFFER_LENGTH){
         observer_vars.buffer[observer_vars.cursor++] = EVENT_FRAME_RX;
         observer_vars.buffer[observer_vars.cursor++] = entity_id;
         observer_vars.buffer[observer_vars.cursor++] = frame_id;
         observer_vars.buffer[observer_vars.cursor++] = data_length >> 0 & 0xff;
         observer_vars.buffer[observer_vars.cursor++] = data_length >> 8 & 0xff;
         observer_vars.buffer[observer_vars.cursor++] = properties_count;
         memcpy(&(observer_vars.buffer[observer_vars.cursor]), data, data_length);
         observer_vars.cursor += data_length;
         observer_vars.properties_left_count = properties_count;
      }else{
         observer_vars.overflow_error = TRUE;
      }
      if (observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_frame_consume(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data){
   /**
   | `entityID` | `frameID` |  `dataLength` |     `data`      |
   |:----------:|:---------:|:-------------:|:---------------:|
   |    8 bits  |   8 bits  |    16 bits    |  variable size  |
   **/
   if( observer_new_event() == E_SUCCESS ){
      if(4 + data_length < OBSERVER_BUFFER_LENGTH){
      observer_vars.buffer[observer_vars.cursor++] = EVENT_FRAME_CONSUME;
      observer_vars.buffer[observer_vars.cursor++] = entity_id;
      observer_vars.buffer[observer_vars.cursor++] = frame_id;
      observer_vars.buffer[observer_vars.cursor++] = data_length >> 0 & 0xff;
      observer_vars.buffer[observer_vars.cursor++] = data_length >> 8 & 0xff;
      memcpy(&(observer_vars.buffer[observer_vars.cursor]), data, data_length);
      observer_vars.cursor += data_length;
      }else{
         observer_vars.overflow_error = TRUE;
      }
      observer_submit();
   }
}


void observer_property_declaration(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint8_t value_type, uint16_t value_length, void* value){
   uint8_t name_length;
   uint8_t content_length;
   /**
   Property Declaration format:

   | `propertyID` | `unitPrefix` | `unit` | `dataType` |  `propertyNameLength` | `propertyValueLength` | `propertyName` | `propertyValue` |
   |:------------:|:------------:|:------:|:----------:|:---------------------:|:---------------------:|:--------------:|:---------------:|
   |     8bits    |    8bits     |  8bits |    8bits   |         8bits         |       16bits          |  variable size |  variable size  |
   **/
   name_length = strlen(name);
   /**
   check if there is enough room in the buffer
      - if yes, add the content to the buffer
      - if not, raise and overflow critical error
   **/
   if(!observer_vars.overflow_error){
      content_length = 7 + name_length + value_length;
      if(observer_vars.cursor + content_length < OBSERVER_BUFFER_LENGTH){
         observer_vars.buffer[observer_vars.cursor++] = id;
         observer_vars.buffer[observer_vars.cursor++] = prefix;
         observer_vars.buffer[observer_vars.cursor++] = unit;
         observer_vars.buffer[observer_vars.cursor++] = value_type;
         observer_vars.buffer[observer_vars.cursor++] = name_length;
         observer_vars.buffer[observer_vars.cursor++] = value_length >> 0 & 0xff;
         observer_vars.buffer[observer_vars.cursor++] = value_length >> 8 & 0xff;
         memcpy(&(observer_vars.buffer[observer_vars.cursor]), name, name_length);
         observer_vars.cursor += name_length;
         memcpy(&(observer_vars.buffer[observer_vars.cursor]), value, value_length);
         observer_vars.cursor += value_length;
      } else {
         observer_vars.overflow_error = TRUE;
      }
      observer_vars.properties_left_count -= 1;
      if(observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}

void observer_property_update(uint8_t id, uint16_t value_length, void* value){
   uint8_t content_length;
   /**
   | `propertyID` | `propertyValueLength` | `propertyValue` |
   |:------------:|:---------------------:|:---------------:|
   |    8 bits    |       16 bits         |  variable size  |
   **/
   /**
   check if there is enough room in the buffer
      - if yes, add the content to the buffer
      - if not, raise and overflow critical error
   **/
   if(!observer_vars.overflow_error){
      content_length = 3 + value_length;
      if(observer_vars.cursor + content_length < OBSERVER_BUFFER_LENGTH){
         observer_vars.buffer[observer_vars.cursor++] = id;
         observer_vars.buffer[observer_vars.cursor++] = value_length >> 0 & 0xff;
         observer_vars.buffer[observer_vars.cursor++] = value_length >> 8 & 0xff;
         memcpy(&(observer_vars.buffer[observer_vars.cursor]), value, value_length);
         observer_vars.cursor += value_length;
      }else{
         observer_vars.overflow_error = TRUE;
      }
      observer_vars.properties_left_count -= 1;
      if(observer_vars.properties_left_count == 0){
         observer_submit();
      }
   }
}


void observer_property_declaration_boolean(uint8_t id, char* name, uint8_t value){
   observer_property_declaration(id, name, PREFIX_NONE, UNIT_NONE, TYPE_BOOLEAN, sizeof(uint8_t), &value);
}

void observer_property_declaration_int8(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int8_t value){
   observer_property_declaration(id, name, prefix, unit, TYPE_INT8, sizeof(int8_t), &value);
}

void observer_property_declaration_uint8(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint8_t value){
   observer_property_declaration(id, name, prefix, unit, TYPE_UINT8, sizeof(uint8_t), &value);
}

void observer_property_declaration_int16(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int16_t value){
   observer_property_declaration(id, name, prefix, unit, TYPE_INT16, sizeof(int16_t), &value);
}

void observer_property_declaration_uint16(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint16_t value){
   observer_property_declaration(id, name, prefix, unit, TYPE_UINT16, sizeof(uint16_t), &value);
}

void observer_property_declaration_int32(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int32_t value){
   observer_property_declaration(id, name, prefix, unit, TYPE_INT32, sizeof(int32_t), &value);
}

void observer_property_declaration_uint32(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint32_t value){
   observer_property_declaration(id, name, prefix, unit, TYPE_UINT32, sizeof(uint32_t), &value);
}

void observer_property_declaration_int64(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int64_t value){
   observer_property_declaration(id, name, prefix, unit, TYPE_INT64, sizeof(int64_t), &value);
}

void observer_property_declaration_uint64(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint64_t value){
   observer_property_declaration(id, name, prefix, unit, TYPE_UINT64, sizeof(uint64_t), &value);
}

void observer_property_declaration_float(uint8_t id, char* name, uint8_t prefix, uint8_t unit, float value){
   observer_property_declaration(id, name, prefix, unit, TYPE_FLOAT, sizeof(float), &value);
}

void observer_property_declaration_double(uint8_t id, char* name, uint8_t prefix, uint8_t unit, double value){
   observer_property_declaration(id, name, prefix, unit, TYPE_DOUBLE, sizeof(double), &value);
}

void observer_property_declaration_ASCII_array(uint8_t id, char* name, uint16_t value_length, char* value){
   observer_property_declaration(id, name, PREFIX_NONE, UNIT_NONE, TYPE_ASCII_ARRAY, value_length, value);
}

void observer_property_declaration_byte_array(uint8_t id, char* name, uint16_t value_length, uint8_t* value){
   observer_property_declaration(id, name, PREFIX_NONE, UNIT_NONE, TYPE_BYTE_ARRAY, value_length, value);
}

void observer_property_update_boolean(uint8_t id, uint8_t value){
   observer_property_update(id, sizeof(uint8_t), &value);
}

void observer_property_update_int8(uint8_t id, int8_t value){
   observer_property_update(id, sizeof(int8_t), &value);
}

void observer_property_update_uint8(uint8_t id, uint8_t value){
   observer_property_update(id, sizeof(uint8_t), &value);
}

void observer_property_update_int16(uint8_t id, int16_t value){
   observer_property_update(id, sizeof(int16_t), &value);
}

void observer_property_update_uint16(uint8_t id, uint16_t value){
   observer_property_update(id, sizeof(uint16_t), &value);
}

void observer_property_update_int32(uint8_t id, int32_t value){
   observer_property_update(id, sizeof(int32_t), &value);
}

void observer_property_update_uint32(uint8_t id, uint32_t value){
   observer_property_update(id, sizeof(uint32_t), &value);
}

void observer_property_update_int64(uint8_t id, int64_t value){
   observer_property_update(id, sizeof(int64_t), &value);
}

void observer_property_update_uint64(uint8_t id, uint64_t value){
   observer_property_update(id, sizeof(uint64_t), &value);
}

void observer_property_update_float(uint8_t id, float value){
   observer_property_update(id, sizeof(float), &value);
}

void observer_property_update_double(uint8_t id, double value){
   observer_property_update(id, sizeof(double), &value);
}

void observer_property_update_ASCII_array(uint8_t id, uint16_t value_length, char* value){
   observer_property_update(id, value_length, value);
}

void observer_property_update_byte_array(uint8_t id, uint16_t value_length, uint8_t* value){
   observer_property_update(id, value_length, value);
}

void observer_property_reference_boolean(uint8_t id, uint8_t value){
   observer_property_update(id, sizeof(uint8_t), &value);
}

void observer_property_reference_int8(uint8_t id, int8_t value){
   observer_property_update(id, sizeof(int8_t), &value);
}

void observer_property_reference_uint8(uint8_t id, uint8_t value){
   observer_property_update(id, sizeof(uint8_t), &value);
}

void observer_property_reference_int16(uint8_t id, int16_t value){
   observer_property_update(id, sizeof(int16_t), &value);
}

void observer_property_reference_uint16(uint8_t id, uint16_t value){
   observer_property_update(id, sizeof(uint16_t), &value);
}

void observer_property_reference_int32(uint8_t id, int32_t value){
   observer_property_update(id, sizeof(int32_t), &value);
}

void observer_property_reference_uint32(uint8_t id, uint32_t value){
   observer_property_update(id, sizeof(uint32_t), &value);
}

void observer_property_reference_int64(uint8_t id, int64_t value){
   observer_property_update(id, sizeof(int64_t), &value);
}

void observer_property_reference_uint64(uint8_t id, uint64_t value){
   observer_property_update(id, sizeof(uint64_t), &value);
}

void observer_property_reference_float(uint8_t id, float value){
   observer_property_update(id, sizeof(float), &value);
}

void observer_property_reference_double(uint8_t id, double value){
   observer_property_update(id, sizeof(double), &value);
}

void observer_property_reference_ASCII_array(uint8_t id, uint16_t value_length, char* value){
   observer_property_update(id, value_length, value);
}

void observer_property_reference_byte_array(uint8_t id, uint16_t value_length, uint8_t* value){
   observer_property_update(id, value_length, value);
}


/* functions specific to OpenWSN */
void owsn_observer_frame_produce(OpenQueueEntry_t *packet, uint8_t properties_count){
   observer_frame_produce(packet->owner, packet->id, packet->length,  packet->payload, properties_count);
}

void owsn_observer_frame_property_add(OpenQueueEntry_t *packet, uint8_t properties_count){
   observer_frame_property_add(packet->owner, packet->id, properties_count);
}

void owsn_observer_frame_property_update(OpenQueueEntry_t *packet, uint8_t properties_count){
   observer_frame_property_update(packet->owner, packet->id, properties_count);
}

void owsn_observer_frame_data_update(OpenQueueEntry_t *packet){
   observer_frame_data_update(packet->owner, packet->id, packet->length, packet->payload);
}

void owsn_observer_frame_tx(OpenQueueEntry_t *packet){
   observer_frame_tx(COMPONENT_RADIO, packet->id, packet->length, packet->payload);
}

void owsn_observer_frame_rx(OpenQueueEntry_t *packet, uint8_t properties_count){
   observer_frame_rx(COMPONENT_RADIO, packet->id, packet->length, packet->payload, properties_count);
}

void owsn_observer_frame_consume(OpenQueueEntry_t *packet){
   observer_frame_consume(packet->owner, packet->id, packet->length, packet->payload);
}

void owsn_observer_link_l1_add(neighborRow_t * neighbor){
   observer_link_add(COMPONENT_RADIO, neighbor->id, 0, 1, 1);
   observer_property_reference_byte_array(PROPERTY_L1_ADDRESS, 8, (uint8_t*)&(neighbor->addr_64b));
   observer_property_declaration_int8(PROPERTY_L1_RSSI, PROPERTY_NAME_L1_RSSI, PREFIX_NONE, UNIT_NONE, neighbor->rssi);
}
void owsn_observer_link_l2_add(neighborRow_t * neighbor){
   observer_link_add(COMPONENT_IEEE802154E, neighbor->id, 0, 1, 5);
   observer_property_reference_byte_array(PROPERTY_L2_NODE_ADDRESS_64B, 8, (uint8_t*)&(neighbor->addr_64b));
   observer_property_declaration_boolean(PROPERTY_L2_LINK_IS_STABLE, PROPERTY_NAME_L2_LINK_IS_STABLE, neighbor->stableNeighbor);
   observer_property_declaration_uint8(PROPERTY_L2_LINK_NUM_STABILITY, PROPERTY_NAME_L2_LINK_NUM_STABILITY, PREFIX_NONE, UNIT_NONE, neighbor->switchStabilityCounter);
   observer_property_declaration_uint8(PROPERTY_L2_LINK_NUM_RX, PROPERTY_NAME_L2_LINK_NUM_RX, PREFIX_NONE, UNIT_NONE, neighbor->numRx);
   observer_property_declaration_uint8(PROPERTY_L2_LINK_NUM_TX, PROPERTY_NAME_L2_LINK_NUM_TX, PREFIX_NONE, UNIT_NONE, neighbor->numTx);
   observer_property_declaration_uint8(PROPERTY_L2_LINK_NUM_TX_ACK, PROPERTY_NAME_L2_LINK_NUM_TX_ACK, PREFIX_NONE, UNIT_NONE, neighbor->numTxACK);
}
void owsn_observer_link_l3_add(neighborRow_t * neighbor){
   uint8_t address[16];

   memcpy(address, idmanager_getMyID(ADDR_PREFIX)->prefix, 8);
   memcpy(address+8, &(neighbor->addr_64b), 8);
   observer_link_add(COMPONENT_ICMPv6RPL, neighbor->id, 0, 1, 3);
   observer_property_reference_byte_array(PROPERTY_L3_NODE_ADDRESS, 16, address);
   observer_property_declaration_uint8(PROPERTY_L3_LINK_PARENT_PREFERENCE, PROPERTY_NAME_L3_LINK_PARENT_PREFERENCE, PREFIX_NONE, UNIT_NONE, neighbor->parentPreference);
   observer_property_declaration_uint16(PROPERTY_L3_LINK_NEIGHBOR_DAGRANK, PROPERTY_NAME_L3_LINK_NEIGHBOR_DAGRANK, PREFIX_NONE, UNIT_NONE, neighbor->DAGrank);
   observer_property_declaration_uint8(PROPERTY_L3_LINK_PARENT_JOIN_PRIORITY, PROPERTY_NAME_L3_LINK_PARENT_JOIN_PRIORITY, PREFIX_NONE, UNIT_NONE, neighbor->joinPrio);
}

void owsn_observer_link_l1_remove(neighborRow_t * neighbor){
   observer_link_remove(COMPONENT_RADIO, neighbor->id);
}

void owsn_observer_link_l2_remove(neighborRow_t * neighbor){
   observer_link_remove(COMPONENT_IEEE802154E, neighbor->id);
}

void owsn_observer_link_l3_remove(neighborRow_t * neighbor){
   observer_link_remove(COMPONENT_ICMPv6RPL, neighbor->id);
}



#else

owerror_t observer_init(void){
   return E_SUCCESS;
}

owerror_t observer_new_event(void){
   return E_SUCCESS;
}




void observer_node_add(uint8_t properties_count){
   
}

void observer_node_property_add(uint8_t properties_count){
   
}

void observer_node_property_update(uint8_t properties_count){
   
}

void observer_node_remove(void){
   
}

void observer_entity_add(uint8_t id, char* name, uint8_t properties_count){
   
}

void observer_entity_property_add(uint8_t id, uint8_t properties_count){
   
}

void observer_entity_property_update(uint8_t id, uint8_t properties_count){

}

void observer_entity_remove(uint8_t id){
   
}

void observer_link_add(uint8_t entity_id, uint8_t link_id, uint8_t src_properties_count, uint8_t tgt_properties_count, uint8_t properties_count){
   
}

void observer_link_property_add(uint8_t entity_id, uint8_t link_id, uint8_t properties_count){
   
}

void observer_link_property_update(uint8_t entity_id, uint8_t link_id, uint8_t properties_count){
   
}

void observer_link_remove(uint8_t entity_id, uint8_t link_id){
   
}

void observer_frame_produce(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data, uint8_t properties_count){
   
}

void observer_frame_property_add(uint8_t entity_id, uint8_t frame_id, uint8_t properties_count){
   
}

void observer_frame_property_update(uint8_t entity_id, uint8_t frame_id, uint8_t properties_count){
   
}

void observer_frame_data_update(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data){
   
}

void observer_frame_tx(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data){
   
}


void observer_frame_rx(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data, uint8_t properties_count){
   
}

void observer_frame_consume(uint8_t entity_id, uint8_t frame_id, uint16_t data_length,  void* data){
   
}


void observer_property_declaration(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint8_t value_type, uint16_t value_length, void* value){

}

void observer_property_update(uint8_t id, uint16_t value_length, void* value){
   
}


void observer_property_declaration_boolean(uint8_t id, char* name, uint8_t value){
   
}

void observer_property_declaration_int8(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int8_t value){
   
}

void observer_property_declaration_uint8(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint8_t value){
   
}

void observer_property_declaration_int16(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int16_t value){
   
}

void observer_property_declaration_uint16(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint16_t value){
  
}

void observer_property_declaration_int32(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int32_t value){
   
}

void observer_property_declaration_uint32(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint32_t value){
  
}

void observer_property_declaration_int64(uint8_t id, char* name, uint8_t prefix, uint8_t unit, int64_t value){
   
}

void observer_property_declaration_uint64(uint8_t id, char* name, uint8_t prefix, uint8_t unit, uint64_t value){
   
}

void observer_property_declaration_float(uint8_t id, char* name, uint8_t prefix, uint8_t unit, float value){
  
}

void observer_property_declaration_double(uint8_t id, char* name, uint8_t prefix, uint8_t unit, double value){
   
}

void observer_property_declaration_ASCII_array(uint8_t id, char* name, uint16_t value_length, char* value){
   
}

void observer_property_declaration_byte_array(uint8_t id, char* name, uint16_t value_length, uint8_t* value){
   
}

void observer_property_update_boolean(uint8_t id, uint8_t value){
   
}

void observer_property_update_int8(uint8_t id, int8_t value){
   
}

void observer_property_update_uint8(uint8_t id, uint8_t value){
   
}

void observer_property_update_int16(uint8_t id, int16_t value){
   
}

void observer_property_update_uint16(uint8_t id, uint16_t value){
   

}

void observer_property_update_int32(uint8_t id, int32_t value){
   
}

void observer_property_update_uint32(uint8_t id, uint32_t value){
   
}

void observer_property_update_int64(uint8_t id, int64_t value){
   
}

void observer_property_update_uint64(uint8_t id, uint64_t value){
   
}

void observer_property_update_float(uint8_t id, float value){
  
}

void observer_property_update_double(uint8_t id, double value){
   
}

void observer_property_update_ASCII_array(uint8_t id, uint16_t value_length, char* value){
   
}

void observer_property_update_byte_array(uint8_t id, uint16_t value_length, uint8_t* value){
   
}

void observer_property_reference_boolean(uint8_t id, uint8_t value){
   
}

void observer_property_reference_int8(uint8_t id, int8_t value){
   
}

void observer_property_reference_uint8(uint8_t id, uint8_t value){
   
}

void observer_property_reference_int16(uint8_t id, int16_t value){
   
}

void observer_property_reference_uint16(uint8_t id, uint16_t value){
   
}

void observer_property_reference_int32(uint8_t id, int32_t value){
   
}

void observer_property_reference_uint32(uint8_t id, uint32_t value){
   
}

void observer_property_reference_int64(uint8_t id, int64_t value){
   
}

void observer_property_reference_uint64(uint8_t id, uint64_t value){
   
}

void observer_property_reference_float(uint8_t id, float value){
   
}

void observer_property_reference_double(uint8_t id, double value){
   
}

void observer_property_reference_ASCII_array(uint8_t id, uint16_t value_length, char* value){
   
}

void observer_property_reference_byte_array(uint8_t id, uint16_t value_length, uint8_t* value){
   
}


/* functions specific to OpenWSN */
void owsn_observer_frame_produce(OpenQueueEntry_t *packet, uint8_t properties_count){
   
}

void owsn_observer_frame_property_add(OpenQueueEntry_t *packet, uint8_t properties_count){
   
}

void owsn_observer_frame_property_update(OpenQueueEntry_t *packet, uint8_t properties_count){
   
}

void owsn_observer_frame_data_update(OpenQueueEntry_t *packet){
   
}

void owsn_observer_frame_tx(OpenQueueEntry_t *packet){
   
}

void owsn_observer_frame_rx(OpenQueueEntry_t *packet, uint8_t properties_count){
   
}

void owsn_observer_frame_consume(OpenQueueEntry_t *packet){
   
}

void owsn_observer_link_l1_add(neighborRow_t * neighbor){
   
}
void owsn_observer_link_l2_add(neighborRow_t * neighbor){
   
}
void owsn_observer_link_l3_add(neighborRow_t * neighbor){
   
}

void owsn_observer_link_l1_remove(neighborRow_t * neighbor){
   
}

void owsn_observer_link_l2_remove(neighborRow_t * neighbor){
   
}

void owsn_observer_link_l3_remove(neighborRow_t * neighbor){
   
}



#endif

