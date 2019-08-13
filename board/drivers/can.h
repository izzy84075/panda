// IRQs: CAN1_TX, CAN1_RX0, CAN1_SCE, CAN2_TX, CAN2_RX0, CAN2_SCE, CAN3_TX, CAN3_RX0, CAN3_SCE

#define CAN_MAX 3

#define can_buffer(x, size) \
  CAN_FIFOMailBox_TypeDef elems_##x[size]; \
  can_ring can_##x = { .w_ptr = 0, .r_ptr = 0, .fifo_size = size, .elems = (CAN_FIFOMailBox_TypeDef *)&elems_##x };

can_buffer(tx1_q, 0x100)
can_buffer(tx2_q, 0x100)
can_buffer(tx3_q, 0x100)
can_buffer(txgmlan_q, 0x100)
can_ring *can_queues[] = {&can_tx1_q, &can_tx2_q, &can_tx3_q, &can_txgmlan_q};

int can_err_cnt = 0;
int can0_mailbox_full_cnt = 0;
int can1_mailbox_full_cnt = 0;
int can0_rx_cnt = 0;
int can2_rx_cnt = 0;
int can0_tx_cnt = 0;
int can2_tx_cnt = 0;

void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number);
bool can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem);


// assign CAN numbering
// bus num: Can bus number on ODB connector. Sent to/from USB
//    Min: 0; Max: 127; Bit 7 marks message as receipt (bus 129 is receipt for but 1)
// cans: Look up MCU can interface from bus number
// can number: numeric lookup for MCU CAN interfaces (0 = CAN1, 1 = CAN2, etc);
// bus_lookup: Translates from 'can number' to 'bus number'.
// can_num_lookup: Translates from 'bus number' to 'can number'.
// can_forwarding: Given a bus num, lookup bus num to forward to. -1 means no forward.

// Panda:       Bus 0=CAN1   Bus 1=CAN2   Bus 2=CAN3
CAN_TypeDef *cans[] = {CAN1, CAN2, CAN3};
uint8_t bus_lookup[] = {0,1,2};
uint8_t can_num_lookup[] = {0,1,2,-1};
int8_t can_forwarding[] = {-1,-1,-1,-1};
uint32_t can_speed[] = {5000, 5000, 5000, 333};

#define AUTOBAUD_SPEEDS_LEN (sizeof(can_autobaud_speeds) / sizeof(can_autobaud_speeds[0]))

#define CANIF_FROM_CAN_NUM(num) (cans[num])
#ifdef PANDA
#define CAN_NUM_FROM_CANIF(CAN) (CAN==CAN1 ? 0 : (CAN==CAN2 ? 1 : 2))
#define CAN_NAME_FROM_CANIF(CAN) (CAN==CAN1 ? "CAN1" : (CAN==CAN2 ? "CAN2" : "CAN3"))
#else
#define CAN_NUM_FROM_CANIF(CAN) (CAN==CAN1 ? 0 : 1)
#define CAN_NAME_FROM_CANIF(CAN) (CAN==CAN1 ? "CAN1" : "CAN2")
#endif
#define BUS_NUM_FROM_CAN_NUM(num) (bus_lookup[num])
#define CAN_NUM_FROM_BUS_NUM(num) (can_num_lookup[num])

// other option
/*#define CAN_QUANTA 16
#define CAN_SEQ1 13
#define CAN_SEQ2 2*/

// this is needed for 1 mbps support
#define CAN_QUANTA 8
#define CAN_SEQ1 6 // roundf(quanta * 0.875f) - 1;
#define CAN_SEQ2 1 // roundf(quanta * 0.125f);

#define CAN_PCLK 24000
// 333 = 33.3 kbps
// 5000 = 500 kbps
#define can_speed_to_prescaler(x) (CAN_PCLK / CAN_QUANTA * 10 / (x))

void can_set_speed(uint8_t can_number) {
  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

  while (true) {
    // initialization mode
    CAN->MCR = CAN_MCR_TTCM | CAN_MCR_INRQ;
    while((CAN->MSR & CAN_MSR_INAK) != CAN_MSR_INAK);

    // set time quanta from defines
    CAN->BTR = (CAN_BTR_TS1_0 * (CAN_SEQ1-1)) |
              (CAN_BTR_TS2_0 * (CAN_SEQ2-1)) |
              (can_speed_to_prescaler(can_speed[bus_number]) - 1);

    // reset
    CAN->MCR = CAN_MCR_TTCM | CAN_MCR_ABOM;

    #define CAN_TIMEOUT 1000000
    int tmp = 0;
    while((CAN->MSR & CAN_MSR_INAK) == CAN_MSR_INAK && tmp < CAN_TIMEOUT) tmp++;
    if (tmp < CAN_TIMEOUT) {
      return;
    }

    puts("CAN init FAILED!!!!!\n");
    puth(can_number); puts(" ");
    puth(BUS_NUM_FROM_CAN_NUM(can_number)); puts("\n");
    return;
  }
}

void can_init(uint8_t can_number) {
  if (can_number == 0xff) return;

  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  set_can_enable(CAN, 1);
  can_set_speed(can_number);

  // accept all filter
  CAN->FMR |= CAN_FMR_FINIT;

  // no mask
  CAN->FS1R|=CAN_FS1R_FSC0;

  if (can_number == 99) {
    CAN->sFilterRegister[0].FR1 = 0x24B<<21;
    CAN->sFilterRegister[0].FR2 = 0xF1<<21;
    CAN->sFilterRegister[1].FR1 = 0xC9<<21;
    CAN->sFilterRegister[1].FR2 = 0x1E9<<21;
    CAN->sFilterRegister[2].FR1 = 0x1C4<<21;
    CAN->sFilterRegister[2].FR2 = 0x1C5<<21;
    CAN->sFilterRegister[3].FR1 = 0x1F5<<21;
    CAN->sFilterRegister[3].FR2 = 0x1E1<<21;
    CAN->sFilterRegister[4].FR1 = 0x214<<21;
    CAN->sFilterRegister[4].FR2 = 0x230<<21;
    CAN->sFilterRegister[5].FR1 = 0x34A<<21;
    CAN->sFilterRegister[5].FR2 = 0x12A<<21;
    CAN->sFilterRegister[6].FR1 = 0x135<<21;
    CAN->sFilterRegister[6].FR2 = 0x184<<21;
    CAN->sFilterRegister[7].FR1 = 0x1F1<<21;
    CAN->sFilterRegister[7].FR2 = 0x140<<21;
    CAN->sFilterRegister[14].FR1 = 0;
    CAN->sFilterRegister[14].FR2 = 0;
    CAN->FM1R |= CAN_FM1R_FBM0 | CAN_FM1R_FBM1 | CAN_FM1R_FBM2 | CAN_FM1R_FBM3 | CAN_FM1R_FBM4 | CAN_FM1R_FBM5 | CAN_FM1R_FBM6 | CAN_FM1R_FBM7;
    CAN->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT1 | CAN_FA1R_FACT2 | CAN_FA1R_FACT3 | CAN_FA1R_FACT4 | CAN_FA1R_FACT5 | CAN_FA1R_FACT6 | CAN_FA1R_FACT7;
    CAN->FA1R |= 1 | (1 << 14);
  }
  else {
    CAN->sFilterRegister[0].FR1 = 0;
    CAN->sFilterRegister[0].FR2 = 0;
    CAN->sFilterRegister[14].FR1 = 0;
    CAN->sFilterRegister[14].FR2 = 0;
    CAN->FA1R |= 1 | (1 << 14);
  }

  CAN->FMR &= ~(CAN_FMR_FINIT);

  // enable certain CAN interrupts
  CAN->IER |= CAN_IER_TMEIE | CAN_IER_FMPIE0;
  //NVIC_EnableIRQ(CAN_IER_TMEIE);
  switch (can_number) {
    case 0:
      NVIC_EnableIRQ(CAN1_TX_IRQn);
      NVIC_EnableIRQ(CAN1_RX0_IRQn);
      NVIC_EnableIRQ(CAN1_SCE_IRQn);
      break;
    case 1:
      NVIC_EnableIRQ(CAN2_TX_IRQn);
      NVIC_EnableIRQ(CAN2_RX0_IRQn);
      NVIC_EnableIRQ(CAN2_SCE_IRQn);
      break;
    case 2:
      NVIC_EnableIRQ(CAN3_TX_IRQn);
      NVIC_EnableIRQ(CAN3_RX0_IRQn);
      NVIC_EnableIRQ(CAN3_SCE_IRQn);
      break;
  }

  // in case there are queued up messages
  //process_can(can_number);
}

int can_overflow_cnt = 0;

// ********************* interrupt safe queue *********************

bool can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem) {
  bool ret = 0;

  enter_critical_section();
  if (q->w_ptr != q->r_ptr) {
    *elem = q->elems[q->r_ptr];
    if ((q->r_ptr + 1) == q->fifo_size) q->r_ptr = 0;
    else q->r_ptr += 1;
    ret = 1;
  }
  exit_critical_section();

  return ret;
}

int can_push(can_ring *q, CAN_FIFOMailBox_TypeDef *elem) {
  int ret = 0;
  uint32_t next_w_ptr;

  enter_critical_section();
  if ((q->w_ptr + 1) == q->fifo_size) next_w_ptr = 0;
  else next_w_ptr = q->w_ptr + 1;
  if (next_w_ptr != q->r_ptr) {
    q->elems[q->w_ptr] = *elem;
    q->w_ptr = next_w_ptr;
    ret = 1;
  }
  exit_critical_section();
  if (ret == 0) {
    can_overflow_cnt++;
    #ifdef DEBUG
      puts("can_push failed!\n");
    #endif
  }
  return ret;
}

void process_can(uint8_t can_number) {
  if (can_number != 0xff) {

    enter_critical_section();

    CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
    uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

    // check for empty mailbox
    CAN_FIFOMailBox_TypeDef to_send;
    if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
      // add successfully transmitted message to my fifo
      if ((CAN->TSR & CAN_TSR_RQCP0) == CAN_TSR_RQCP0) {
        //can_txd_cnt += 1;

        //if ((CAN->TSR & CAN_TSR_TXOK0) == CAN_TSR_TXOK0) {
        //  CAN_FIFOMailBox_TypeDef to_push;
        //  to_push.RIR = CAN->sTxMailBox[0].TIR;
        //  to_push.RDTR = (CAN->sTxMailBox[0].TDTR & 0xFFFF000F) | ((CAN_BUS_RET_FLAG | bus_number) << 4);
        //  to_push.RDLR = CAN->sTxMailBox[0].TDLR;
        //  to_push.RDHR = CAN->sTxMailBox[0].TDHR;
        //  can_push(&can_rx_q, &to_push);
        //}

        if ((CAN->TSR & CAN_TSR_TERR0) == CAN_TSR_TERR0) {
          #ifdef DEBUG
            puts("CAN TX ERROR!\n");
          #endif
        }

        if ((CAN->TSR & CAN_TSR_ALST0) == CAN_TSR_ALST0) {
          #ifdef DEBUG
            puts("CAN TX ARBITRATION LOST!\n");
          #endif
        }

        // clear interrupt
        // careful, this can also be cleared by requesting a transmission
        CAN->TSR |= CAN_TSR_RQCP0;
      }

      if (can_pop(can_queues[bus_number], &to_send)) {
        //can_tx_cnt += 1;
        // only send if we have received a packet
        CAN->sTxMailBox[0].TDLR = to_send.RDLR;
        CAN->sTxMailBox[0].TDHR = to_send.RDHR;
        CAN->sTxMailBox[0].TDTR = to_send.RDTR;
        CAN->sTxMailBox[0].TIR = to_send.RIR;

	if (can_number == 0) {
          can0_tx_cnt++;
        }
        if (can_number == 2) {
          can2_tx_cnt++;
        }
      }
    }

    exit_critical_section();
  }
}

void can_init_all() {
  for (int i=0; i < CAN_MAX; i++) {
    can_init(i);
  }
}

// single wire (low speed) GMLAN only used for button presses to change mode in the future
void can_set_gmlan(int bus) {
  if (bus == -1 || bus != can_num_lookup[3]) {
    // GMLAN OFF
    switch (can_num_lookup[3]) {
      case 1:
        puts("disable GMLAN on CAN2\n");
        set_can_mode(1, 0);
        bus_lookup[1] = 1;
        can_num_lookup[1] = 1;
        can_num_lookup[3] = -1;
        can_init(1);
        break;
      case 2:
        puts("disable GMLAN on CAN3\n");
        set_can_mode(2, 0);
        bus_lookup[2] = 2;
        can_num_lookup[2] = 2;
        can_num_lookup[3] = -1;
        can_init(2);
        break;
    }
  }

  if (bus == 1) {
    puts("GMLAN on CAN2\n");
    // GMLAN on CAN2
    set_can_mode(1, 1);
    bus_lookup[1] = 3;
    can_num_lookup[1] = -1;
    can_num_lookup[3] = 1;
    can_init(1);
  } else if (bus == 2 && revision == PANDA_REV_C) {
    puts("GMLAN on CAN3\n");
    // GMLAN on CAN3
    set_can_mode(2, 1);
    bus_lookup[2] = 3;
    can_num_lookup[2] = -1;
    can_num_lookup[3] = 2;
    can_init(2);
  }
}

// CAN error
void can_sce(CAN_TypeDef *CAN) {
  enter_critical_section();

  can_err_cnt += 1;
  if (CAN==CAN1) puts("CAN1:  ");
  if (CAN==CAN2) puts("CAN2:  ");
  if (CAN==CAN3) puts("CAN3:  ");
  puts("MSR:");
  puth(CAN->MSR);
  puts(" TSR:");
  puth(CAN->TSR);
  puts(" RF0R:");
  puth(CAN->RF0R);
  puts(" RF1R:");
  puth(CAN->RF1R);
  puts(" ESR:");
  puth(CAN->ESR);
  puts("\n");

  // clear current send
  CAN->TSR |= CAN_TSR_ABRQ0;
  CAN->MSR &= ~(CAN_MSR_ERRI);
  CAN->MSR = CAN->MSR;

  exit_critical_section();
}

// ***************************** CAN *****************************

int fwd_filter(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  uint32_t addr = to_fwd->RIR>>21;

  if (bus_num == 0) {


// ######### PT ###################
    if ((addr == 0x24B) ||
        (addr == 0xF1) ||
	(addr == 0xC9) ||
	(addr == 0x1E9) ||
	(addr == 0x1C4) ||
	(addr == 0x1C5) ||
	(addr == 0x1F5) ||
	(addr == 0x1E1) ||
	(addr == 0x214) ||
	(addr == 0x230) ||
	(addr == 0x34A) ||
	(addr == 0x12A) ||
	(addr == 0x135) ||
	(addr == 0x184) ||
	(addr == 0x1F1) ||
	(addr == 0x140)) {
      return 2;
    }
    return -1;

    //puts("\n");
    //puth(addr);

//    if ((addr == 0xC1) || (addr == 0xC5) || (addr == 0xD1) || (addr == 0x1C8) || (addr == 0x1E5) || (addr == 0xBE)
//       || (addr == 0xC7) || (addr == 0xD3) || (addr == 0xF9) || (addr == 0x18E)|| (addr == 0x191) || (addr == 0x1E7)
//       || (addr == 0x1EB) || (addr == 0x1ED) || (addr == 0x1C7) || (addr == 0x1CE) || (addr == 0x189)
//       || (addr == 0x197) || (addr == 0x19D) || (addr == 0x1A1) || (addr == 0x1A3) || (addr == 0x1A6) || (addr == 0x1AA)
//       || (addr == 0x1AF) || (addr == 0x1BA) || (addr == 0x1C3) || (addr == 0xC4) || (addr == 0x1DF) || (addr == 0x287)
//       || (addr == 0x321) || (addr == 0x1F3) || (addr == 0x2CD) || (addr == 0x2CF) || (addr == 0x365)
//       || (addr == 0x372) || (addr == 0x1F4) || (addr == 0x1FC) || (addr == 0x1FE) || (addr == 0x210) || (addr == 0x232)
//       || (addr == 0x2C3) || (addr == 0x2D3) || (addr == 0x29F) || (addr == 0x348) || (addr == 0x34C) || (addr == 0x362)
//       || (addr == 0x137) || (addr == 0x139) || (addr == 0x142)  || (addr == 0x160)  || (addr == 0x170)
//       || (addr == 0x17D) || (addr == 0x182) || (addr == 0x216) 
//       || (addr == 0x22A) || (addr == 0x234) || (addr == 0x260) || (addr == 0x261) || (addr == 0x262) || (addr == 0x263) || (addr == 0x264)
//       || (addr == 0x265) || (addr == 0x2F1) || (addr == 0x32A) || (addr == 0x37A) || (addr == 0x37C) || (addr == 0x37D) || (addr == 0x3C1) || (addr == 0x3C7)
//       || (addr == 0x3C7) || (addr == 0x3C9) || (addr == 0x3D1) || (addr == 0x3D3) || (addr == 0x3E9) || (addr == 0x409)
//       || (addr == 0x40A) || (addr == 0x451) || (addr == 0x510) || (addr == 0x120) || (addr == 0x130)
//       || (addr == 0x148) || (addr == 0x233) || (addr == 0x235) || (addr == 0x237) || (addr == 0x23D) || (addr == 0x241)
//       || (addr == 0x324) || (addr == 0x37E) || (addr == 0x3ED) || (addr == 0x3F1) || (addr == 0x3F3) || (addr == 0x3F5)
//       || (addr == 0x3F9) || (addr == 0x3FB) || (addr == 0x3FC) || (addr == 0x4C1) || (addr == 0x4C5) || (addr == 0x4C7)
//       || (addr == 0x4C9) || (addr == 0x4D1) || (addr == 0x4D9) || (addr == 0x4E1) || (addr == 0x4E9) || (addr == 0x4EB)
//       || (addr == 0x4ED) || (addr == 0x4EF) || (addr == 0x4F1) || (addr == 0x4F3) || (addr == 0x4F7) || (addr == 0x500)
//       || (addr == 0x514) || (addr == 0x52A) || (addr == 0x52B) || (addr == 0x530) || (addr == 0x589) || (addr == 0x641)
//       || (addr == 0x652) || (addr == 0x770) || (addr == 0x772) || (addr == 0x773) || (addr == 0x778) || (addr == 0x77C)
//       || (addr == 0x77D) || (addr == 0x77E) || (addr == 0x77F) || (addr == 0x780) || (addr == 0x78A) || (addr == 0x7E0) || (addr == 0x7E8)
//       || (addr == 0x2F9) 
//      ) {
//      return -1;
//    }
//    puts("\n");
//    puth(addr);
//    return 2; 


// ########### END PT ###############


// ######### Chassis ####################
//    if ((addr == 0xC0) || (addr == 0xC1) || (addr == 0xC5) || (addr == 0x130) || (addr == 0x140) || (addr == 0x170) || (addr == 0x1E5)) {
//      return 2;
//    }
//    return -1;
    //if ((addr == 0xC9) || (addr == 0x160) || (addr == 0x323) || (addr == 0x180) || (addr == 0x1CE) || (addr == 0x1D5) || (addr == 0x1E9)
    //     || (addr == 0x212) || (addr == 0x234) || (addr == 0x335) || (addr == 0x320) || (addr == 0x321) || (addr == 0x325)
//	 || (addr == 0x1FC) || (addr == 0x348) || (addr == 0x34A) || (addr == 0x362) || (addr == 0x365) || (addr == 0x121)
//	 || (addr == 0x122) || (addr == 0x12A) || (addr == 0x12C) || (addr == 0x17D) || (addr == 0x182) || (addr == 0x1CA)
//	 || (addr == 0x1F5) || (addr == 0x200) || (addr == 0x238) || (addr == 0x340) || (addr == 0x3E9) || (addr == 0x3EB) || (addr == 0x148)
//	 || (addr == 0x4E1) || (addr == 0x514)          || (addr == 0xC0)) {
//      return -1; 
//    }
//    puts("\n");
//    puth(addr);
//    return 2;

// ########## end chassis ###############

  }

  if (bus_num == 2) {
    // Drop all steering + brake messages
    //if ((addr == 384) || (addr == 715)  || (addr == 789) || (addr == 880)) {

    // Drop all steering messages
    if ((addr == 384)) {
        return -1;
    }

    return 0;
  }

  return -1;
}

//void can_tx(uint8_t can_number, CAN_FIFOMailBox_TypeDef *to_tx) {
//  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
//  uint8_t transmitmailbox;
//
//  /* Check that all the Tx mailioxes are not full */
//  if (((CAN->TSR & CAN_TSR_TME0) != RESET) ||
//      ((CAN->TSR & CAN_TSR_TME1) != RESET) ||
//      ((CAN->TSR & CAN_TSR_TME2) != RESET))
//  {
//    transmitmailbox = (CAN->TSR & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
//
//    CAN->sTxMailBox[transmitmailbox].TDLR = to_tx->RDLR;
//    CAN->sTxMailBox[transmitmailbox].TDHR = to_tx->RDHR;
//    CAN->sTxMailBox[transmitmailbox].TDTR = to_tx->RDTR;
//    CAN->sTxMailBox[transmitmailbox].TIR = to_tx->RIR;
//
//    if (can_number == 0) {
//      can0_tx_cnt++;
//    }
//    if (can_number == 2) {
//      can2_tx_cnt++;
//    }
//    return;
//  }
//  if (can_number == 0) {
//    can0_mailbox_full_cnt++;
//  }
//  if (can_number == 2) {
//    can0_mailbox_full_cnt++;
//  }
//}

// CAN receive handlers
// blink blue when we are receiving CAN messages
void can_rx(uint8_t can_number) {
  //enter_critical_section();
  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);
  while (CAN->RF0R & CAN_RF0R_FMP0) {
    if (can_number == 0) {
      can0_rx_cnt++;
    }
    if (can_number == 2) {
      can2_rx_cnt++;
    }

    // add to my fifo
    CAN_FIFOMailBox_TypeDef to_tx;
    to_tx.RIR = CAN->sFIFOMailBox[0].RIR | 1; //TXRQ
    to_tx.RDTR = CAN->sFIFOMailBox[0].RDTR;
    to_tx.RDLR = CAN->sFIFOMailBox[0].RDLR;
    to_tx.RDHR = CAN->sFIFOMailBox[0].RDHR;

    // modify RDTR for our API
    to_tx.RDTR = (to_tx.RDTR & 0xFFFF000F) | (bus_number << 4);

    // forwarding
    int bus_fwd_num = fwd_filter(bus_number, &to_tx);
    if (bus_fwd_num != -1) {
      //can_tx(bus_fwd_num, &to_tx);
      can_push(can_queues[bus_number], &to_tx);
      process_can(CAN_NUM_FROM_BUS_NUM(bus_number));
    }

    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
  //exit_critical_section();
}

void CAN1_TX_IRQHandler(void) { process_can(0); }
void CAN1_RX0_IRQHandler() { can_rx(0); }
void CAN1_SCE_IRQHandler() { can_sce(CAN1); }

void CAN2_TX_IRQHandler(void) { process_can(1); }
void CAN2_RX0_IRQHandler() { can_rx(1); }
void CAN2_SCE_IRQHandler() { can_sce(CAN2); }

void CAN3_TX_IRQHandler(void) { process_can(2); }
void CAN3_RX0_IRQHandler() { can_rx(2); }
void CAN3_SCE_IRQHandler() { can_sce(CAN3); }
