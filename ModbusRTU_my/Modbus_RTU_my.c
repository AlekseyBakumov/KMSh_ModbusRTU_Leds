#include "Modbus_RTU_my.h"

void init_ModbusRTU_Handle(ModbusRTU_Handle_t* handle, uint8_t id) {
  handle->self_address = id;
  handle->response_length = 0;
}

void set_Read_Handler(ModbusRTU_Handle_t* handle, ModbusReadCallback handler) {
  handle->user_read_callback = handler;
}

void set_Write_Handler(ModbusRTU_Handle_t* handle, ModbusWriteCallback handler) {
  handle->user_write_callback = handler;
}

void load_uart_message(ModbusRTU_Handle_t* handle, uint8_t* buffer, uint16_t len) {
  memcpy(handle->current_request.original_message, buffer, len);
  handle->current_request.orig_len = len;
  handle->current_request.request_processed = 0;
}

void parse_from_uart_message(ModbusRTU_Package_t* package) {
  package->address   = package->original_message[0];
  package->func_code = package->original_message[1];
  package->CRC16     = (uint16_t)(package->original_message[package->orig_len-1] << 8 | package->original_message[package->orig_len-2]);
  memcpy(package->data, package->original_message+2, package->orig_len-4);
  package->data_len = package->orig_len-4;
  package->request_processed = 0;
}

uint16_t eval_CRC16(uint8_t* buffer, uint16_t len) {
    uint16_t crc = CRC16_val;
    for (int i = 0; i < len; i++) {
      crc ^= (uint16_t)buffer[i];
      for (int j = 0; j < 8; j++) {
        if (crc & 0x0001) {
          crc = (crc >> 1) ^ 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    
    return crc;
}

void functions_0x01_0x02(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len, uint8_t enable0x02) {
  uint16_t start_addr = (input[0] << 8) | input[1];
  uint16_t count      = (input[2] << 8) | input[3];
  
  uint8_t out_bytes = 1 + (uint8_t)((count-1)/8);
  uint16_t read_bits = 0;
  
  uint16_t val = 0;
  uint8_t error = 0;
  
  output[(*out_len)++] = out_bytes;
  
  if (handle->user_read_callback == NULL) return; // Need to add notification to user about null callback.
  
  for (int i = 0; i < out_bytes; i++) {
    uint8_t temp_byte = 0;
    for (int j = 0; j < 8; j++) {
      if (read_bits >= count) break;
      val = 0;
      error = handle->user_read_callback(start_addr+(i*8)+j, &val, enable0x02 ? READ_MODE_DISCRETE_INP : READ_MODE_COILS);
      read_bits++;

      if (error != 0) {
        *out_len = 2;
        output[1] += 0x80;
        output[(*out_len)++] = error;
        return;
      }
      
      if (val > 0) {
        temp_byte |= (1 << j);
      }
    }
    
    output[(*out_len)++] = temp_byte;
  }
}

void process_Function_0x01(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len) {
  functions_0x01_0x02(handle, input, in_len, output, out_len, 0);
}

void process_Function_0x02(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len) {
  functions_0x01_0x02(handle, input, in_len, output, out_len, 1);
}

void functions_0x03_0x04(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len, uint8_t enable0x04) {
  uint16_t start_addr = (input[0] << 8) | input[1];
  uint16_t count      = (input[2] << 8) | input[3];
  
  output[(*out_len)++] = (uint8_t)count * 2;
  
  for (int i = 0; i < count; i++) {
    uint16_t current_addr = start_addr + i;
    
    uint16_t value = 0;
    uint8_t error = 0;
    
    if (handle->user_read_callback != NULL)
      error = handle->user_read_callback(current_addr, &value, enable0x04 ? READ_MODE_INPUT_REGS : READ_MODE_HOLDING_REGS);
    
    if (error != 0) {
        *out_len = 2;
        output[1] += 0x80;
        output[(*out_len)++] = error;
        return;
      }
    
    output[(*out_len)++] = (value >> 8) & 0xFF;
    output[(*out_len)++] =  value       & 0xFF;
  }
}

void process_Function_0x03(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len) {
  functions_0x03_0x04(handle, input, in_len, output, out_len, 0);
}

void process_Function_0x04(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len) {
  functions_0x03_0x04(handle, input, in_len, output, out_len, 1);
}

void process_Function_0x05(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len) {
  uint16_t start_addr = (input[0] << 8) | input[1];
  uint16_t value      = (input[2] << 8) | input[3];
  
  uint8_t error = 0;
  
  if (handle->user_write_callback != NULL)
      error = handle->user_write_callback(start_addr, value, WRITE_MODE_SINGLE_COIL);
  
  if (error != 0) {
    output[1] += 0x80;
    output[(*out_len)++] = error;
    return;
  }
  
  output[(*out_len)++] = (start_addr >> 8) & 0xFF;
  output[(*out_len)++] =  start_addr       & 0xFF;
  
  output[(*out_len)++] = (value >> 8) & 0xFF;
  output[(*out_len)++] =  value       & 0xFF;  
}

void process_Function_0x06(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len) {
  uint16_t addr  = (input[0] << 8) | input[1];
  uint16_t value = (input[2] << 8) | input[3];
  uint8_t error = 0;
  
  if (handle->user_write_callback != NULL)
    error = handle->user_write_callback(addr, value, WRITE_MODE_SINGLE_REG);
  
  if (error != 0) {
    output[1] += 0x80;
    output[(*out_len)++] = error;
    return;
  }
  
  output[(*out_len)++] = (addr >> 8) & 0xFF;
  output[(*out_len)++] =  addr       & 0xFF;
  
  output[(*out_len)++] = (value >> 8) & 0xFF;
  output[(*out_len)++] =  value       & 0xFF;  
}

void process_Function_0x0F(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len) {
  uint16_t start_addr = (input[0] << 8) | input[1];
  uint16_t count      = (input[2] << 8) | input[3];
  uint8_t bytes_input = input[4];
  
  uint16_t written_bits = 0;
  uint8_t error = 0;
   
  uint8_t coil_val = 0;
  
  output[(*out_len)++] = (start_addr >> 8) & 0xFF;
  output[(*out_len)++] =  start_addr       & 0xFF;
  
  if (handle->user_write_callback == NULL) return; // Need to add notification to user about null callback.
  
  for (int i = 0; i < bytes_input; i++) {
    for (int j = 0; j < 8; j++) {
      if (written_bits >= count) break;
      coil_val = (input[5+i] & (1 << j));
      error = handle->user_write_callback(start_addr+(i*8)+j, coil_val ? 0xFF00 : 0x0000, WRITE_MODE_SINGLE_COIL);
      
      if (error != 0) {
        *out_len = 2;
        output[1] += 0x80;
        output[(*out_len)++] = error;
        return;
      }
      
      written_bits++;
    }
  }
  
  output[(*out_len)++] = (written_bits >> 8) & 0xFF;
  output[(*out_len)++] =  written_bits       & 0xFF;
}

void process_Function_0x10(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len) {
  uint16_t start_addr = (input[0] << 8) | input[1];
  uint16_t count      = (input[2] << 8) | input[3];
  uint8_t bytes_input = input[4];
  
  uint16_t written_2bits = 0;
  uint16_t new_val = 0;
  uint8_t error = 0;
  
  output[(*out_len)++] = (start_addr >> 8) & 0xFF;
  output[(*out_len)++] =  start_addr       & 0xFF;
  
  if (handle->user_write_callback == NULL) return; // Need to add notification to user about null callback.
  
  for (int i = 0; i < count; i++) {
    new_val = (input[5 + 2 * i] << 8) | input[6 + 2 * i];
    error = handle->user_write_callback(start_addr + i, new_val, WRITE_MODE_SINGLE_REG);
    
    if (error != 0) {
      *out_len = 2;
      output[1] += 0x80;
      output[(*out_len)++] = error;
      return;
    }
      
    written_2bits++;
  }
  
  output[(*out_len)++] = (written_2bits >> 8) & 0xFF;
  output[(*out_len)++] =  written_2bits       & 0xFF;
}

uint8_t process_Request(ModbusRTU_Handle_t* handle) {
  if (handle->current_request.request_processed) return PROCESS_DOUBLE_CALL;
  
  parse_from_uart_message(&(handle->current_request));
  handle->response_length = 0;
  
  if (handle->current_request.address != handle->self_address) {
    handle->current_request.request_processed = 1;
    return PROCESS_WRONG_ID;
  }
  
  if (eval_CRC16(handle->current_request.original_message, handle->current_request.orig_len-2) != handle->current_request.CRC16) {
    handle->current_request.request_processed = 1;
    return PROCESS_WRONG_CRC;
  }
  
  handle->response_buffer[handle->response_length++] = handle->self_address;
  handle->response_buffer[handle->response_length++] = handle->current_request.func_code;
  
  switch(handle->current_request.func_code) {
  case 0x01:
    process_Function_0x01(handle, handle->current_request.data, handle->current_request.data_len, handle->response_buffer, &(handle->response_length));
    break;
  case 0x02:
    process_Function_0x02(handle, handle->current_request.data, handle->current_request.data_len, handle->response_buffer, &(handle->response_length));
    break;
  case 0x03:
    process_Function_0x03(handle, handle->current_request.data, handle->current_request.data_len, handle->response_buffer, &(handle->response_length));
    break;
  case 0x04:
    process_Function_0x04(handle, handle->current_request.data, handle->current_request.data_len, handle->response_buffer, &(handle->response_length));
    break;
  case 0x05:
    process_Function_0x05(handle, handle->current_request.data, handle->current_request.data_len, handle->response_buffer, &(handle->response_length));
    break;
  case 0x06:
    process_Function_0x06(handle, handle->current_request.data, handle->current_request.data_len, handle->response_buffer, &(handle->response_length));
    break;
  case 0x0F:
    process_Function_0x0F(handle, handle->current_request.data, handle->current_request.data_len, handle->response_buffer, &(handle->response_length));
    break;
  case 0x10:
    process_Function_0x10(handle, handle->current_request.data, handle->current_request.data_len, handle->response_buffer, &(handle->response_length));
    break;
    
  default:
    return PROCESS_FUNC_CODE_UKNW;
  }

  uint16_t new_crc = eval_CRC16(handle->response_buffer, handle->response_length);
  handle->response_buffer[handle->response_length++] = new_crc & 0xFF;
  handle->response_buffer[handle->response_length++] = (new_crc >> 8) & 0xFF;
  
  return PROCESS_OK;
}

uint8_t* get_Output(ModbusRTU_Handle_t* handle, uint8_t* len) {
  (*len) = handle->response_length;
  return handle->response_buffer;
}