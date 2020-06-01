/* AUTOGENERATED FILE. DO NOT EDIT. */
#include <string.h>
#include <stdlib.h>
#include <setjmp.h>
#include "cmock.h"
#include "mock_scu_18xx_43xx.h"

static const char* CMockString_ADC_ID = "ADC_ID";
static const char* CMockString_Chip_SCU_ADC_Channel_Config = "Chip_SCU_ADC_Channel_Config";
static const char* CMockString_Chip_SCU_ClockPinMux = "Chip_SCU_ClockPinMux";
static const char* CMockString_Chip_SCU_ClockPinMuxSet = "Chip_SCU_ClockPinMuxSet";
static const char* CMockString_Chip_SCU_DAC_Analog_Config = "Chip_SCU_DAC_Analog_Config";
static const char* CMockString_Chip_SCU_GPIOIntPinSel = "Chip_SCU_GPIOIntPinSel";
static const char* CMockString_Chip_SCU_I2C0PinConfig = "Chip_SCU_I2C0PinConfig";
static const char* CMockString_Chip_SCU_PinMux = "Chip_SCU_PinMux";
static const char* CMockString_Chip_SCU_PinMuxSet = "Chip_SCU_PinMuxSet";
static const char* CMockString_Chip_SCU_SetPinMuxing = "Chip_SCU_SetPinMuxing";
static const char* CMockString_I2C0Mode = "I2C0Mode";
static const char* CMockString_PinNum = "PinNum";
static const char* CMockString_PortNum = "PortNum";
static const char* CMockString_PortSel = "PortSel";
static const char* CMockString_arrayLength = "arrayLength";
static const char* CMockString_channel = "channel";
static const char* CMockString_clknum = "clknum";
static const char* CMockString_func = "func";
static const char* CMockString_mode = "mode";
static const char* CMockString_modefunc = "modefunc";
static const char* CMockString_pin = "pin";
static const char* CMockString_pinArray = "pinArray";
static const char* CMockString_port = "port";

typedef struct _CMOCK_Chip_SCU_PinMuxSet_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  int CallOrder;
  uint8_t Expected_port;
  uint8_t Expected_pin;
  uint16_t Expected_modefunc;

} CMOCK_Chip_SCU_PinMuxSet_CALL_INSTANCE;

typedef struct _CMOCK_Chip_SCU_PinMux_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  int CallOrder;
  uint8_t Expected_port;
  uint8_t Expected_pin;
  uint16_t Expected_mode;
  uint8_t Expected_func;

} CMOCK_Chip_SCU_PinMux_CALL_INSTANCE;

typedef struct _CMOCK_Chip_SCU_ClockPinMuxSet_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  int CallOrder;
  uint8_t Expected_clknum;
  uint16_t Expected_modefunc;

} CMOCK_Chip_SCU_ClockPinMuxSet_CALL_INSTANCE;

typedef struct _CMOCK_Chip_SCU_ClockPinMux_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  int CallOrder;
  uint8_t Expected_clknum;
  uint16_t Expected_mode;
  uint8_t Expected_func;

} CMOCK_Chip_SCU_ClockPinMux_CALL_INSTANCE;

typedef struct _CMOCK_Chip_SCU_GPIOIntPinSel_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  int CallOrder;
  uint8_t Expected_PortSel;
  uint8_t Expected_PortNum;
  uint8_t Expected_PinNum;

} CMOCK_Chip_SCU_GPIOIntPinSel_CALL_INSTANCE;

typedef struct _CMOCK_Chip_SCU_I2C0PinConfig_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  int CallOrder;
  uint32_t Expected_I2C0Mode;

} CMOCK_Chip_SCU_I2C0PinConfig_CALL_INSTANCE;

typedef struct _CMOCK_Chip_SCU_ADC_Channel_Config_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  int CallOrder;
  uint32_t Expected_ADC_ID;
  uint8_t Expected_channel;

} CMOCK_Chip_SCU_ADC_Channel_Config_CALL_INSTANCE;

typedef struct _CMOCK_Chip_SCU_DAC_Analog_Config_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  int CallOrder;

} CMOCK_Chip_SCU_DAC_Analog_Config_CALL_INSTANCE;

typedef struct _CMOCK_Chip_SCU_SetPinMuxing_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  int CallOrder;
  const PINMUX_GRP_T* Expected_pinArray;
  uint32_t Expected_arrayLength;

} CMOCK_Chip_SCU_SetPinMuxing_CALL_INSTANCE;

static struct mock_scu_18xx_43xxInstance
{
  char Chip_SCU_PinMuxSet_IgnoreBool;
  CMOCK_MEM_INDEX_TYPE Chip_SCU_PinMuxSet_CallInstance;
  char Chip_SCU_PinMux_IgnoreBool;
  CMOCK_MEM_INDEX_TYPE Chip_SCU_PinMux_CallInstance;
  char Chip_SCU_ClockPinMuxSet_IgnoreBool;
  CMOCK_MEM_INDEX_TYPE Chip_SCU_ClockPinMuxSet_CallInstance;
  char Chip_SCU_ClockPinMux_IgnoreBool;
  CMOCK_MEM_INDEX_TYPE Chip_SCU_ClockPinMux_CallInstance;
  char Chip_SCU_GPIOIntPinSel_IgnoreBool;
  CMOCK_MEM_INDEX_TYPE Chip_SCU_GPIOIntPinSel_CallInstance;
  char Chip_SCU_I2C0PinConfig_IgnoreBool;
  CMOCK_MEM_INDEX_TYPE Chip_SCU_I2C0PinConfig_CallInstance;
  char Chip_SCU_ADC_Channel_Config_IgnoreBool;
  CMOCK_MEM_INDEX_TYPE Chip_SCU_ADC_Channel_Config_CallInstance;
  char Chip_SCU_DAC_Analog_Config_IgnoreBool;
  CMOCK_MEM_INDEX_TYPE Chip_SCU_DAC_Analog_Config_CallInstance;
  char Chip_SCU_SetPinMuxing_IgnoreBool;
  CMOCK_MEM_INDEX_TYPE Chip_SCU_SetPinMuxing_CallInstance;
} Mock;

extern jmp_buf AbortFrame;
extern int GlobalExpectCount;
extern int GlobalVerifyOrder;

void mock_scu_18xx_43xx_Verify(void)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_MEM_INDEX_TYPE call_instance;
  call_instance = Mock.Chip_SCU_PinMuxSet_CallInstance;
  if (Mock.Chip_SCU_PinMuxSet_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_Chip_SCU_PinMuxSet);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  call_instance = Mock.Chip_SCU_PinMux_CallInstance;
  if (Mock.Chip_SCU_PinMux_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_Chip_SCU_PinMux);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  call_instance = Mock.Chip_SCU_ClockPinMuxSet_CallInstance;
  if (Mock.Chip_SCU_ClockPinMuxSet_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_Chip_SCU_ClockPinMuxSet);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  call_instance = Mock.Chip_SCU_ClockPinMux_CallInstance;
  if (Mock.Chip_SCU_ClockPinMux_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_Chip_SCU_ClockPinMux);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  call_instance = Mock.Chip_SCU_GPIOIntPinSel_CallInstance;
  if (Mock.Chip_SCU_GPIOIntPinSel_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_Chip_SCU_GPIOIntPinSel);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  call_instance = Mock.Chip_SCU_I2C0PinConfig_CallInstance;
  if (Mock.Chip_SCU_I2C0PinConfig_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_Chip_SCU_I2C0PinConfig);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  call_instance = Mock.Chip_SCU_ADC_Channel_Config_CallInstance;
  if (Mock.Chip_SCU_ADC_Channel_Config_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_Chip_SCU_ADC_Channel_Config);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  call_instance = Mock.Chip_SCU_DAC_Analog_Config_CallInstance;
  if (Mock.Chip_SCU_DAC_Analog_Config_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_Chip_SCU_DAC_Analog_Config);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  call_instance = Mock.Chip_SCU_SetPinMuxing_CallInstance;
  if (Mock.Chip_SCU_SetPinMuxing_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_Chip_SCU_SetPinMuxing);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
}

void mock_scu_18xx_43xx_Init(void)
{
  mock_scu_18xx_43xx_Destroy();
}

void mock_scu_18xx_43xx_Destroy(void)
{
  CMock_Guts_MemFreeAll();
  memset(&Mock, 0, sizeof(Mock));
  GlobalExpectCount = 0;
  GlobalVerifyOrder = 0;
}

void Chip_SCU_PinMuxSet(uint8_t port, uint8_t pin, uint16_t modefunc)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_Chip_SCU_PinMuxSet_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_Chip_SCU_PinMuxSet);
  cmock_call_instance = (CMOCK_Chip_SCU_PinMuxSet_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.Chip_SCU_PinMuxSet_CallInstance);
  Mock.Chip_SCU_PinMuxSet_CallInstance = CMock_Guts_MemNext(Mock.Chip_SCU_PinMuxSet_CallInstance);
  if (Mock.Chip_SCU_PinMuxSet_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    return;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_PinMuxSet,CMockString_port);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_port, port, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_PinMuxSet,CMockString_pin);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_pin, pin, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_PinMuxSet,CMockString_modefunc);
    UNITY_TEST_ASSERT_EQUAL_HEX16(cmock_call_instance->Expected_modefunc, modefunc, cmock_line, CMockStringMismatch);
  }
  UNITY_CLR_DETAILS();
}

void CMockExpectParameters_Chip_SCU_PinMuxSet(CMOCK_Chip_SCU_PinMuxSet_CALL_INSTANCE* cmock_call_instance, uint8_t port, uint8_t pin, uint16_t modefunc)
{
  cmock_call_instance->Expected_port = port;
  cmock_call_instance->Expected_pin = pin;
  cmock_call_instance->Expected_modefunc = modefunc;
}

void Chip_SCU_PinMuxSet_CMockIgnore(void)
{
  Mock.Chip_SCU_PinMuxSet_IgnoreBool = (char)1;
}

void Chip_SCU_PinMuxSet_CMockStopIgnore(void)
{
  Mock.Chip_SCU_PinMuxSet_IgnoreBool = (char)0;
}

void Chip_SCU_PinMuxSet_CMockExpect(UNITY_LINE_TYPE cmock_line, uint8_t port, uint8_t pin, uint16_t modefunc)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_Chip_SCU_PinMuxSet_CALL_INSTANCE));
  CMOCK_Chip_SCU_PinMuxSet_CALL_INSTANCE* cmock_call_instance = (CMOCK_Chip_SCU_PinMuxSet_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.Chip_SCU_PinMuxSet_CallInstance = CMock_Guts_MemChain(Mock.Chip_SCU_PinMuxSet_CallInstance, cmock_guts_index);
  Mock.Chip_SCU_PinMuxSet_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_Chip_SCU_PinMuxSet(cmock_call_instance, port, pin, modefunc);
}

void Chip_SCU_PinMux(uint8_t port, uint8_t pin, uint16_t mode, uint8_t func)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_Chip_SCU_PinMux_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_Chip_SCU_PinMux);
  cmock_call_instance = (CMOCK_Chip_SCU_PinMux_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.Chip_SCU_PinMux_CallInstance);
  Mock.Chip_SCU_PinMux_CallInstance = CMock_Guts_MemNext(Mock.Chip_SCU_PinMux_CallInstance);
  if (Mock.Chip_SCU_PinMux_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    return;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_PinMux,CMockString_port);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_port, port, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_PinMux,CMockString_pin);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_pin, pin, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_PinMux,CMockString_mode);
    UNITY_TEST_ASSERT_EQUAL_HEX16(cmock_call_instance->Expected_mode, mode, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_PinMux,CMockString_func);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_func, func, cmock_line, CMockStringMismatch);
  }
  UNITY_CLR_DETAILS();
}

void CMockExpectParameters_Chip_SCU_PinMux(CMOCK_Chip_SCU_PinMux_CALL_INSTANCE* cmock_call_instance, uint8_t port, uint8_t pin, uint16_t mode, uint8_t func)
{
  cmock_call_instance->Expected_port = port;
  cmock_call_instance->Expected_pin = pin;
  cmock_call_instance->Expected_mode = mode;
  cmock_call_instance->Expected_func = func;
}

void Chip_SCU_PinMux_CMockIgnore(void)
{
  Mock.Chip_SCU_PinMux_IgnoreBool = (char)1;
}

void Chip_SCU_PinMux_CMockStopIgnore(void)
{
  Mock.Chip_SCU_PinMux_IgnoreBool = (char)0;
}

void Chip_SCU_PinMux_CMockExpect(UNITY_LINE_TYPE cmock_line, uint8_t port, uint8_t pin, uint16_t mode, uint8_t func)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_Chip_SCU_PinMux_CALL_INSTANCE));
  CMOCK_Chip_SCU_PinMux_CALL_INSTANCE* cmock_call_instance = (CMOCK_Chip_SCU_PinMux_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.Chip_SCU_PinMux_CallInstance = CMock_Guts_MemChain(Mock.Chip_SCU_PinMux_CallInstance, cmock_guts_index);
  Mock.Chip_SCU_PinMux_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_Chip_SCU_PinMux(cmock_call_instance, port, pin, mode, func);
}

void Chip_SCU_ClockPinMuxSet(uint8_t clknum, uint16_t modefunc)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_Chip_SCU_ClockPinMuxSet_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_Chip_SCU_ClockPinMuxSet);
  cmock_call_instance = (CMOCK_Chip_SCU_ClockPinMuxSet_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.Chip_SCU_ClockPinMuxSet_CallInstance);
  Mock.Chip_SCU_ClockPinMuxSet_CallInstance = CMock_Guts_MemNext(Mock.Chip_SCU_ClockPinMuxSet_CallInstance);
  if (Mock.Chip_SCU_ClockPinMuxSet_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    return;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_ClockPinMuxSet,CMockString_clknum);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_clknum, clknum, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_ClockPinMuxSet,CMockString_modefunc);
    UNITY_TEST_ASSERT_EQUAL_HEX16(cmock_call_instance->Expected_modefunc, modefunc, cmock_line, CMockStringMismatch);
  }
  UNITY_CLR_DETAILS();
}

void CMockExpectParameters_Chip_SCU_ClockPinMuxSet(CMOCK_Chip_SCU_ClockPinMuxSet_CALL_INSTANCE* cmock_call_instance, uint8_t clknum, uint16_t modefunc)
{
  cmock_call_instance->Expected_clknum = clknum;
  cmock_call_instance->Expected_modefunc = modefunc;
}

void Chip_SCU_ClockPinMuxSet_CMockIgnore(void)
{
  Mock.Chip_SCU_ClockPinMuxSet_IgnoreBool = (char)1;
}

void Chip_SCU_ClockPinMuxSet_CMockStopIgnore(void)
{
  Mock.Chip_SCU_ClockPinMuxSet_IgnoreBool = (char)0;
}

void Chip_SCU_ClockPinMuxSet_CMockExpect(UNITY_LINE_TYPE cmock_line, uint8_t clknum, uint16_t modefunc)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_Chip_SCU_ClockPinMuxSet_CALL_INSTANCE));
  CMOCK_Chip_SCU_ClockPinMuxSet_CALL_INSTANCE* cmock_call_instance = (CMOCK_Chip_SCU_ClockPinMuxSet_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.Chip_SCU_ClockPinMuxSet_CallInstance = CMock_Guts_MemChain(Mock.Chip_SCU_ClockPinMuxSet_CallInstance, cmock_guts_index);
  Mock.Chip_SCU_ClockPinMuxSet_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_Chip_SCU_ClockPinMuxSet(cmock_call_instance, clknum, modefunc);
}

void Chip_SCU_ClockPinMux(uint8_t clknum, uint16_t mode, uint8_t func)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_Chip_SCU_ClockPinMux_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_Chip_SCU_ClockPinMux);
  cmock_call_instance = (CMOCK_Chip_SCU_ClockPinMux_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.Chip_SCU_ClockPinMux_CallInstance);
  Mock.Chip_SCU_ClockPinMux_CallInstance = CMock_Guts_MemNext(Mock.Chip_SCU_ClockPinMux_CallInstance);
  if (Mock.Chip_SCU_ClockPinMux_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    return;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_ClockPinMux,CMockString_clknum);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_clknum, clknum, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_ClockPinMux,CMockString_mode);
    UNITY_TEST_ASSERT_EQUAL_HEX16(cmock_call_instance->Expected_mode, mode, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_ClockPinMux,CMockString_func);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_func, func, cmock_line, CMockStringMismatch);
  }
  UNITY_CLR_DETAILS();
}

void CMockExpectParameters_Chip_SCU_ClockPinMux(CMOCK_Chip_SCU_ClockPinMux_CALL_INSTANCE* cmock_call_instance, uint8_t clknum, uint16_t mode, uint8_t func)
{
  cmock_call_instance->Expected_clknum = clknum;
  cmock_call_instance->Expected_mode = mode;
  cmock_call_instance->Expected_func = func;
}

void Chip_SCU_ClockPinMux_CMockIgnore(void)
{
  Mock.Chip_SCU_ClockPinMux_IgnoreBool = (char)1;
}

void Chip_SCU_ClockPinMux_CMockStopIgnore(void)
{
  Mock.Chip_SCU_ClockPinMux_IgnoreBool = (char)0;
}

void Chip_SCU_ClockPinMux_CMockExpect(UNITY_LINE_TYPE cmock_line, uint8_t clknum, uint16_t mode, uint8_t func)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_Chip_SCU_ClockPinMux_CALL_INSTANCE));
  CMOCK_Chip_SCU_ClockPinMux_CALL_INSTANCE* cmock_call_instance = (CMOCK_Chip_SCU_ClockPinMux_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.Chip_SCU_ClockPinMux_CallInstance = CMock_Guts_MemChain(Mock.Chip_SCU_ClockPinMux_CallInstance, cmock_guts_index);
  Mock.Chip_SCU_ClockPinMux_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_Chip_SCU_ClockPinMux(cmock_call_instance, clknum, mode, func);
}

void Chip_SCU_GPIOIntPinSel(uint8_t PortSel, uint8_t PortNum, uint8_t PinNum)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_Chip_SCU_GPIOIntPinSel_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_Chip_SCU_GPIOIntPinSel);
  cmock_call_instance = (CMOCK_Chip_SCU_GPIOIntPinSel_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.Chip_SCU_GPIOIntPinSel_CallInstance);
  Mock.Chip_SCU_GPIOIntPinSel_CallInstance = CMock_Guts_MemNext(Mock.Chip_SCU_GPIOIntPinSel_CallInstance);
  if (Mock.Chip_SCU_GPIOIntPinSel_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    return;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_GPIOIntPinSel,CMockString_PortSel);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_PortSel, PortSel, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_GPIOIntPinSel,CMockString_PortNum);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_PortNum, PortNum, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_GPIOIntPinSel,CMockString_PinNum);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_PinNum, PinNum, cmock_line, CMockStringMismatch);
  }
  UNITY_CLR_DETAILS();
}

void CMockExpectParameters_Chip_SCU_GPIOIntPinSel(CMOCK_Chip_SCU_GPIOIntPinSel_CALL_INSTANCE* cmock_call_instance, uint8_t PortSel, uint8_t PortNum, uint8_t PinNum)
{
  cmock_call_instance->Expected_PortSel = PortSel;
  cmock_call_instance->Expected_PortNum = PortNum;
  cmock_call_instance->Expected_PinNum = PinNum;
}

void Chip_SCU_GPIOIntPinSel_CMockIgnore(void)
{
  Mock.Chip_SCU_GPIOIntPinSel_IgnoreBool = (char)1;
}

void Chip_SCU_GPIOIntPinSel_CMockStopIgnore(void)
{
  Mock.Chip_SCU_GPIOIntPinSel_IgnoreBool = (char)0;
}

void Chip_SCU_GPIOIntPinSel_CMockExpect(UNITY_LINE_TYPE cmock_line, uint8_t PortSel, uint8_t PortNum, uint8_t PinNum)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_Chip_SCU_GPIOIntPinSel_CALL_INSTANCE));
  CMOCK_Chip_SCU_GPIOIntPinSel_CALL_INSTANCE* cmock_call_instance = (CMOCK_Chip_SCU_GPIOIntPinSel_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.Chip_SCU_GPIOIntPinSel_CallInstance = CMock_Guts_MemChain(Mock.Chip_SCU_GPIOIntPinSel_CallInstance, cmock_guts_index);
  Mock.Chip_SCU_GPIOIntPinSel_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_Chip_SCU_GPIOIntPinSel(cmock_call_instance, PortSel, PortNum, PinNum);
}

void Chip_SCU_I2C0PinConfig(uint32_t I2C0Mode)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_Chip_SCU_I2C0PinConfig_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_Chip_SCU_I2C0PinConfig);
  cmock_call_instance = (CMOCK_Chip_SCU_I2C0PinConfig_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.Chip_SCU_I2C0PinConfig_CallInstance);
  Mock.Chip_SCU_I2C0PinConfig_CallInstance = CMock_Guts_MemNext(Mock.Chip_SCU_I2C0PinConfig_CallInstance);
  if (Mock.Chip_SCU_I2C0PinConfig_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    return;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_I2C0PinConfig,CMockString_I2C0Mode);
    UNITY_TEST_ASSERT_EQUAL_UINT32(cmock_call_instance->Expected_I2C0Mode, I2C0Mode, cmock_line, CMockStringMismatch);
  }
  UNITY_CLR_DETAILS();
}

void CMockExpectParameters_Chip_SCU_I2C0PinConfig(CMOCK_Chip_SCU_I2C0PinConfig_CALL_INSTANCE* cmock_call_instance, uint32_t I2C0Mode)
{
  cmock_call_instance->Expected_I2C0Mode = I2C0Mode;
}

void Chip_SCU_I2C0PinConfig_CMockIgnore(void)
{
  Mock.Chip_SCU_I2C0PinConfig_IgnoreBool = (char)1;
}

void Chip_SCU_I2C0PinConfig_CMockStopIgnore(void)
{
  Mock.Chip_SCU_I2C0PinConfig_IgnoreBool = (char)0;
}

void Chip_SCU_I2C0PinConfig_CMockExpect(UNITY_LINE_TYPE cmock_line, uint32_t I2C0Mode)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_Chip_SCU_I2C0PinConfig_CALL_INSTANCE));
  CMOCK_Chip_SCU_I2C0PinConfig_CALL_INSTANCE* cmock_call_instance = (CMOCK_Chip_SCU_I2C0PinConfig_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.Chip_SCU_I2C0PinConfig_CallInstance = CMock_Guts_MemChain(Mock.Chip_SCU_I2C0PinConfig_CallInstance, cmock_guts_index);
  Mock.Chip_SCU_I2C0PinConfig_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_Chip_SCU_I2C0PinConfig(cmock_call_instance, I2C0Mode);
}

void Chip_SCU_ADC_Channel_Config(uint32_t ADC_ID, uint8_t channel)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_Chip_SCU_ADC_Channel_Config_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_Chip_SCU_ADC_Channel_Config);
  cmock_call_instance = (CMOCK_Chip_SCU_ADC_Channel_Config_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.Chip_SCU_ADC_Channel_Config_CallInstance);
  Mock.Chip_SCU_ADC_Channel_Config_CallInstance = CMock_Guts_MemNext(Mock.Chip_SCU_ADC_Channel_Config_CallInstance);
  if (Mock.Chip_SCU_ADC_Channel_Config_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    return;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_ADC_Channel_Config,CMockString_ADC_ID);
    UNITY_TEST_ASSERT_EQUAL_UINT32(cmock_call_instance->Expected_ADC_ID, ADC_ID, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_ADC_Channel_Config,CMockString_channel);
    UNITY_TEST_ASSERT_EQUAL_HEX8(cmock_call_instance->Expected_channel, channel, cmock_line, CMockStringMismatch);
  }
  UNITY_CLR_DETAILS();
}

void CMockExpectParameters_Chip_SCU_ADC_Channel_Config(CMOCK_Chip_SCU_ADC_Channel_Config_CALL_INSTANCE* cmock_call_instance, uint32_t ADC_ID, uint8_t channel)
{
  cmock_call_instance->Expected_ADC_ID = ADC_ID;
  cmock_call_instance->Expected_channel = channel;
}

void Chip_SCU_ADC_Channel_Config_CMockIgnore(void)
{
  Mock.Chip_SCU_ADC_Channel_Config_IgnoreBool = (char)1;
}

void Chip_SCU_ADC_Channel_Config_CMockStopIgnore(void)
{
  Mock.Chip_SCU_ADC_Channel_Config_IgnoreBool = (char)0;
}

void Chip_SCU_ADC_Channel_Config_CMockExpect(UNITY_LINE_TYPE cmock_line, uint32_t ADC_ID, uint8_t channel)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_Chip_SCU_ADC_Channel_Config_CALL_INSTANCE));
  CMOCK_Chip_SCU_ADC_Channel_Config_CALL_INSTANCE* cmock_call_instance = (CMOCK_Chip_SCU_ADC_Channel_Config_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.Chip_SCU_ADC_Channel_Config_CallInstance = CMock_Guts_MemChain(Mock.Chip_SCU_ADC_Channel_Config_CallInstance, cmock_guts_index);
  Mock.Chip_SCU_ADC_Channel_Config_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_Chip_SCU_ADC_Channel_Config(cmock_call_instance, ADC_ID, channel);
}

void Chip_SCU_DAC_Analog_Config(void)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_Chip_SCU_DAC_Analog_Config_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_Chip_SCU_DAC_Analog_Config);
  cmock_call_instance = (CMOCK_Chip_SCU_DAC_Analog_Config_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.Chip_SCU_DAC_Analog_Config_CallInstance);
  Mock.Chip_SCU_DAC_Analog_Config_CallInstance = CMock_Guts_MemNext(Mock.Chip_SCU_DAC_Analog_Config_CallInstance);
  if (Mock.Chip_SCU_DAC_Analog_Config_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    return;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  UNITY_CLR_DETAILS();
}

void Chip_SCU_DAC_Analog_Config_CMockIgnore(void)
{
  Mock.Chip_SCU_DAC_Analog_Config_IgnoreBool = (char)1;
}

void Chip_SCU_DAC_Analog_Config_CMockStopIgnore(void)
{
  Mock.Chip_SCU_DAC_Analog_Config_IgnoreBool = (char)0;
}

void Chip_SCU_DAC_Analog_Config_CMockExpect(UNITY_LINE_TYPE cmock_line)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_Chip_SCU_DAC_Analog_Config_CALL_INSTANCE));
  CMOCK_Chip_SCU_DAC_Analog_Config_CALL_INSTANCE* cmock_call_instance = (CMOCK_Chip_SCU_DAC_Analog_Config_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.Chip_SCU_DAC_Analog_Config_CallInstance = CMock_Guts_MemChain(Mock.Chip_SCU_DAC_Analog_Config_CallInstance, cmock_guts_index);
  Mock.Chip_SCU_DAC_Analog_Config_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
}

void Chip_SCU_SetPinMuxing(const PINMUX_GRP_T* pinArray, uint32_t arrayLength)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_Chip_SCU_SetPinMuxing_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_Chip_SCU_SetPinMuxing);
  cmock_call_instance = (CMOCK_Chip_SCU_SetPinMuxing_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.Chip_SCU_SetPinMuxing_CallInstance);
  Mock.Chip_SCU_SetPinMuxing_CallInstance = CMock_Guts_MemNext(Mock.Chip_SCU_SetPinMuxing_CallInstance);
  if (Mock.Chip_SCU_SetPinMuxing_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    return;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_SetPinMuxing,CMockString_pinArray);
    UNITY_TEST_ASSERT_EQUAL_MEMORY((void*)(cmock_call_instance->Expected_pinArray), (void*)(pinArray), sizeof(const PINMUX_GRP_T), cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_Chip_SCU_SetPinMuxing,CMockString_arrayLength);
    UNITY_TEST_ASSERT_EQUAL_UINT32(cmock_call_instance->Expected_arrayLength, arrayLength, cmock_line, CMockStringMismatch);
  }
  UNITY_CLR_DETAILS();
}

void CMockExpectParameters_Chip_SCU_SetPinMuxing(CMOCK_Chip_SCU_SetPinMuxing_CALL_INSTANCE* cmock_call_instance, const PINMUX_GRP_T* pinArray, uint32_t arrayLength)
{
  cmock_call_instance->Expected_pinArray = pinArray;
  cmock_call_instance->Expected_arrayLength = arrayLength;
}

void Chip_SCU_SetPinMuxing_CMockIgnore(void)
{
  Mock.Chip_SCU_SetPinMuxing_IgnoreBool = (char)1;
}

void Chip_SCU_SetPinMuxing_CMockStopIgnore(void)
{
  Mock.Chip_SCU_SetPinMuxing_IgnoreBool = (char)0;
}

void Chip_SCU_SetPinMuxing_CMockExpect(UNITY_LINE_TYPE cmock_line, const PINMUX_GRP_T* pinArray, uint32_t arrayLength)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_Chip_SCU_SetPinMuxing_CALL_INSTANCE));
  CMOCK_Chip_SCU_SetPinMuxing_CALL_INSTANCE* cmock_call_instance = (CMOCK_Chip_SCU_SetPinMuxing_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.Chip_SCU_SetPinMuxing_CallInstance = CMock_Guts_MemChain(Mock.Chip_SCU_SetPinMuxing_CallInstance, cmock_guts_index);
  Mock.Chip_SCU_SetPinMuxing_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_Chip_SCU_SetPinMuxing(cmock_call_instance, pinArray, arrayLength);
}
