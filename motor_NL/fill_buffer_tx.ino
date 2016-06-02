uint8_t fill_bufferTx(uint8_t inst) {
  uint8_t num;
  switch (inst) {
    case 0x01:                              // PING
      bufferTx[3] = 0x02;
      bufferTx[4] = error;
      num = _crcTx(bufferTx, 0x02);
      write_flag = true;
      break;
    case 0x02:                              // GOAL_POSITION
      rx_pnt32 = (float *)&bufferRx[5];
      goal_position = *rx_pnt32;
      bufferTx[3] = 0x02;
      bufferTx[4] = error;
      num = _crcTx(bufferTx, 0x02);
      write_flag = true;
      break;
    case 0x03:                              // GOAL_VELOCITY
    {
      // Read the velocity reference
      int *p = (int *)&bufferRx[5];
      vel_reference = *p;
      // Respond to Odroid
      bufferTx[3] = 0x02;
      bufferTx[4] = error;
      num = _crcTx(bufferTx, 0x02);
      write_flag = true;
      break;
    }
    case 0x11:                              // FEEDBACK3
      bufferTx[3] = 14;
      bufferTx[4] = error;
      for (uint8_t i=0;i<4;i++) {
        bufferTx[5+i] = *(p+i);
        bufferTx[9+i] = *(v+i);
        bufferTx[13+i] = *(c+i);
      }
      num = _crcTx(bufferTx, 14);

      write_flag = true;
      break;
    case 0x21:                              // SET_GAIN(P)
      // Update the gain
      rx_pnt32 = (float *)&bufferRx[5];
      _PGAIN = *rx_pnt32;
      bufferTx[3] = 0x02;
      bufferTx[4] = error;
      num = _crcTx(bufferTx, 0x02);
      write_flag = true;
      break;
    case 0x22:                              // SET_GAIN(I)
      // Update the gain
      rx_pnt32 = (float *)&bufferRx[5];
      _DGAIN = *rx_pnt32;
      bufferTx[3] = 0x02;
      bufferTx[4] = error;
      num = _crcTx(bufferTx, 0x02);
      write_flag = true;
      break;
    case 0x23:                              // SET_GAIN(D)
      // Update the gain
      rx_pnt32 = (float *)&bufferRx[5];
      _IGAIN = *rx_pnt32;
      bufferTx[3] = 0x02;
      bufferTx[4] = error;
      num = _crcTx(bufferTx, 0x02);
      write_flag = true;
      break;
    case 0x24:                              // DUTY
      //update the OCR0B
      u = bufferRx[5];
      bufferTx[3] = 0x02; //LEN
      bufferTx[4] = error;
      num = _crcTx(bufferTx, 0x02);
      write_flag = true;
      break;
    case 0x25:                              //GET_VEL
      //send the velocity back
      bufferTx[3] = 0x04;
      bufferTx[4] = error;
      bufferTx[5] = *v;
      bufferTx[6] = *(v+1);
      num = _crcTx(bufferTx, 0x04);
      write_flag = true;
      break;
    case 0x26:                              //GET_POS
      // send the position feedback
      bufferTx[3] = 0x04;
      bufferTx[4] = error;
      bufferTx[5] = *p;
      bufferTx[6] = *(p+1);
      num = _crcTx(bufferTx, 0x04);
      write_flag = true;
      break;

  }

  return num;
}
