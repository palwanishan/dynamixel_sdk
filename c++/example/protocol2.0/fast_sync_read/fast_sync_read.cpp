// Fast Sync Read Example Environment
// - Youtube : https://youtu.be/claLIK8omIQ
//
// - DYNAMIXEL: X series (except XL-320), MX(2.0) series, P series
//              ID = 1, 2, Baudrate = 57600bps, Protocol 2.0
// - USB-Serial Interface : U2D2 (DYNAMIXEL Starter Set)
// - Library: DYNAMIXEL SDK v3.8.1 or later
//
// Author: Will Son


#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>
// Use DYNAMIXEL SDK library
#include "dynamixel_sdk.h"

#define ADDR_TORQUE_ENABLE  64
#define ADDR_GOAL_POSITION  116
#define ADDR_PRESENT_POSITION  132

#define LEN_GOAL_POSITION  4
#define LEN_PRESENT_POSITION  4

#define PROTOCOL_VERSION  2.0

#define DXL1_ID  3
#define DXL2_ID  4
// Most DYNAMIXEL has a default baudrate of 57600
#define BAUDRATE  1000000
// Check which serial port is assigned to the U2D2
// ex) Windows: "COM*"   Linux: "/dev/ttyUSB*" Mac: "/dev/tty.usbserial-*"
#define DEVICENAME  "/dev/ttyUSB0"
// Value for enabling the torque
#define TORQUE_ENABLE  1
// Value for disabling the torque
#define TORQUE_DISABLE  0
// Minimum & Maximum range of Goal Position.
// Invalid value range will be ignored. Refer to the product eManual.
#define DXL_MINIMUM_POSITION_VALUE  0
#define DXL_MAXIMUM_POSITION_VALUE  1023
// Moving status flag threshold
#define DXL_MOVING_STATUS_THRESHOLD  20

#define ESC_ASCII_VALUE  0x1b

int getch()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

int main()
{
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

  int index = 0;
  // Save the communication result
  int dxl_comm_result = COMM_TX_FAIL;
  // Save the AddParam result
  bool dxl_addparam_result = false;
  // Save the GetParam result
  bool dxl_getdata_result = false;
  // Min and Max Goal positions to iterate
  int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };

  uint8_t dxl_error = 0;
  uint8_t param_goal_position[4];
  int32_t dxl1_present_position = 0, dxl2_present_position = 0;

  // Open the serial port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to set the baudrate!\n");
  }
  else
  {
    printf("Failed to set the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable DYNAMIXEL#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(    portHandler,    DXL1_ID,    ADDR_TORQUE_ENABLE,    TORQUE_ENABLE,    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  else if (dxl_error != 0)                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  else                                    printf("DYNAMIXEL#%d has been successfully connected \n", DXL1_ID);

  // Enable DYNAMIXEL#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(    portHandler,   DXL2_ID,    ADDR_TORQUE_ENABLE,    TORQUE_ENABLE,    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  else if (dxl_error != 0)                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  else                                    printf("DYNAMIXEL#%d has been successfully connected \n", DXL2_ID);

  // Add parameter storage for DYNAMIXEL#1 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL1_ID);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL1_ID);
    return 0;
  }

  // Add parameter storage for DYNAMIXEL#2 present position value
  dxl_addparam_result = groupSyncRead.addParam(DXL2_ID);
  if (dxl_addparam_result != true)
  {
    fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL2_ID);
    return 0;
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Allocate goal position value into byte array
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]));

    // Add DYNAMIXEL#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
      return 0;
    }

    // Add DYNAMIXEL#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
      return 0;
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    do
    {
      // Syncread present position
      dxl_comm_result = groupSyncRead.fastSyncReadTxRxPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      if (groupSyncRead.getError(DXL1_ID, &dxl_error))
      {
        printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
      }
      if (groupSyncRead.getError(DXL2_ID, &dxl_error))
      {
        printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
      }

      // Check if groupsyncread data of DYNAMIXEL#1 is available
      dxl_getdata_result = groupSyncRead.isAvailable(        DXL1_ID,        ADDR_PRESENT_POSITION,        LEN_PRESENT_POSITION);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL1_ID);
        return 0;
      }

      // Check if groupsyncread data of DYNAMIXEL#2 is available
      dxl_getdata_result = groupSyncRead.isAvailable(        DXL2_ID,        ADDR_PRESENT_POSITION,        LEN_PRESENT_POSITION);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL2_ID);
        return 0;
      }

      // Get DYNAMIXEL#1 present position value
      dxl1_present_position = groupSyncRead.getData(        DXL1_ID,        ADDR_PRESENT_POSITION,        LEN_PRESENT_POSITION);

      // Get DYNAMIXEL#2 present position value
      dxl2_present_position = groupSyncRead.getData(        DXL2_ID,        ADDR_PRESENT_POSITION,        LEN_PRESENT_POSITION);

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n",
        DXL1_ID,
        dxl_goal_position[index],
        dxl1_present_position,
        DXL2_ID,
        dxl_goal_position[index],
        dxl2_present_position);

    }while(
      (abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) ||
      (abs(dxl_goal_position[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // Change goal position
    if (index == 0) index = 1;
    else            index = 0;
  }

  // Disable DYNAMIXEL#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(    portHandler,    DXL1_ID,    ADDR_TORQUE_ENABLE,    TORQUE_DISABLE,    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  else if (dxl_error != 0)                printf("%s\n", packetHandler->getRxPacketError(dxl_error));

  // Disable DYNAMIXEL#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(    portHandler,    DXL2_ID,    ADDR_TORQUE_ENABLE,    TORQUE_DISABLE,    &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  else if (dxl_error != 0)                printf("%s\n", packetHandler->getRxPacketError(dxl_error));

  // Close port
  portHandler->closePort();

  return 0;
}
