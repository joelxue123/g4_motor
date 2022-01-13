#include "TinyFrame.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>

bool is_sending = false;
uint8_t uart1_tx_buf[256] = {0};

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* phuart_)
{
  if(phuart_ == &huart1)
  {
      is_sending = false;
  }
}

/**
 * This is an example of integrating TinyFrame into the application.
 * 
 * TF_WriteImpl() is required, the mutex functions are weak and can
 * be removed if not used. They are called from all TF_Send/Respond functions.
 * 
 * Also remember to periodically call TF_Tick() if you wish to use the 
 * listener timeout feature.
 */

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
    // send to UART
    is_sending = true; 
    memcpy((void*)uart1_tx_buf, (const void*)buff, len);
    HAL_UART_Transmit_DMA(&huart1, uart1_tx_buf, len);
}

// --------- Mutex callbacks ----------
// Needed only if TF_USE_MUTEX is 1 in the config file.
// DELETE if mutex is not used

/** Claim the TX interface before composing and sending a frame */
bool TF_ClaimTx(TinyFrame *tf)
{
    // take mutex
    if(is_sending) return false;

    return true; // we succeeded
}

/** Free the TX interface after composing and sending a frame */
void TF_ReleaseTx(TinyFrame *tf)
{
    // release mutex
}

// --------- Custom checksums ---------
// This should be defined here only if a custom checksum type is used.
// DELETE those if you use one of the built-in checksum types

/** Initialize a checksum */
TF_CKSUM TF_CksumStart(void)
{
    return 0;
}

/** Update a checksum with a byte */
TF_CKSUM TF_CksumAdd(TF_CKSUM cksum, uint8_t byte)
{
    return cksum ^ byte;
}

/** Finalize the checksum calculation */
TF_CKSUM TF_CksumEnd(TF_CKSUM cksum)
{
    return cksum;
}
