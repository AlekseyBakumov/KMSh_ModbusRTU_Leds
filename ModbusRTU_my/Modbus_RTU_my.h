#ifndef Modbus_RTU_my
#define Modbus_RTU_my

#include <stdint.h>
#include <string.h>

#define CRC16_val 0xFFFF

#define PROCESS_OK              0 // Error codes for modbus processing
#define PROCESS_DOUBLE_CALL     1
#define PROCESS_WRONG_ID        2
#define PROCESS_WRONG_CRC       3

#define PROCESS_FUNC_CODE_UKNW  242


#define READ_MODE_COILS         0x01 // Modes for read callback
#define READ_MODE_DISCRETE_INP  0x02
#define READ_MODE_HOLDING_REGS  0x03
#define READ_MODE_INPUT_REGS    0x04

#define WRITE_MODE_SINGLE_COIL  0x05 // Modes for write callback
#define WRITE_MODE_SINGLE_REG   0x06
#define WRITE_MODE_MULT_COILS   0x0F
#define WRITE_MODE_MULT_REGS    0x10

typedef uint8_t (*ModbusReadCallback)(uint16_t address, uint16_t* value, uint8_t mode);
typedef uint8_t (*ModbusWriteCallback)(uint16_t address, uint16_t value, uint8_t mode);

typedef struct {
  uint8_t address;
  uint8_t func_code;
  uint8_t data[252];
  uint8_t data_len;
  uint16_t CRC16;
  
  uint8_t original_message[256];
  uint8_t orig_len;
  
  uint8_t request_processed;   // Flag, set as 1 if response if already processed
} ModbusRTU_Package_t;


typedef struct {
  uint8_t self_address;                         // Device address
  uint8_t response_buffer[256];                 // Response buffer
  uint8_t response_length;                      // Response buffer length
  ModbusReadCallback user_read_callback;        // Read register callback
  ModbusWriteCallback user_write_callback;      // Write register callback
  ModbusRTU_Package_t current_request;
} ModbusRTU_Handle_t;

/**
  * @brief Init function for new handle
  * @retval void
  */
void init_ModbusRTU_Handle(ModbusRTU_Handle_t* handle, uint8_t id);

/**
  * @brief Set callback for reading user`s registers
  * @retval void
  */
void set_Read_Handler(ModbusRTU_Handle_t* handle, ModbusReadCallback handler);

/**
  * @brief Set callback for writing user`s registers
  * @retval void
  */
void set_Write_Handler(ModbusRTU_Handle_t* handle, ModbusWriteCallback handler);

/**
  * @brief  Copy uart buffer into structure buffer for further processing
  * @retval void
  */
void load_uart_message(ModbusRTU_Handle_t* handle, uint8_t* buffer, uint16_t len);

/**
  * @brief  Divides input uart message from "original_buffer" into struct "output"
  * @retval void
  */
void parse_from_uart_message(ModbusRTU_Package_t* package);

/**
  * @brief  Evaluation of CRC16 checksum.
  * @retval uint16_t
  */
uint16_t eval_CRC16(uint8_t* buffer, uint16_t len);



//Next list of function works with payload (buffer+2, len), and put result directly into output buffer, incrementing out_len

/**
  * @brief  Function 0x01. Read coils.
  */
void process_Function_0x01(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len);
/**
  * @brief  Function 0x02. Read discrete inputs.
  */
void process_Function_0x02(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len);
/**
  * @brief  Function 0x03. Read holding registers.
  */
void process_Function_0x03(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len);
/**
  * @brief  Function 0x04. Read input registers.
  */
void process_Function_0x04(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len); 
/**
  * @brief  Function 0x05. Write single coil.
  */
void process_Function_0x05(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len);
/**
  * @brief  Function 0x06. Write single register.
  */
void process_Function_0x06(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len);
/**
  * @brief  Function 0x0F. Write multiple coils.
  */
void process_Function_0x0F(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len);
/**
  * @brief  Function 0x10. Write multiple registers.
  */
void process_Function_0x10(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len);

void functions_0x01_0x02(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len, uint8_t enable0x02);
void functions_0x03_0x04(ModbusRTU_Handle_t* handle, uint8_t* input, uint8_t in_len, uint8_t* output, uint8_t* out_len, uint8_t enable0x04);


/**
  * @brief  ModbusRTU request processing pipeline
  * @retval uint8_t (error code)
  */
uint8_t process_Request(ModbusRTU_Handle_t* handle);

uint8_t* get_Output(ModbusRTU_Handle_t* handle, uint8_t* len);

#endif