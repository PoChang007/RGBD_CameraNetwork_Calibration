// Copyright 2017 University of Kentucky
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

#include "DepthCodec.h"

#define XN_MAX_UINT16 65535

// Obtained by running xn::DepthGenerator.GetDeviceMaxDepth() on an actual kinect
#define KINECT_MAX_DEPTH 10000

int DepthCodec::encode(const unsigned short *pInput,
					   const unsigned long nInputSize,
					   unsigned char *pOutput,
					   unsigned long *pnOutputSize)
{
	// Local function variables
	const unsigned short *pInputEnd = pInput + (nInputSize / sizeof(unsigned short));
	const unsigned short *pOrigInput = pInput;
	const unsigned char *pOrigOutput = pOutput;
	unsigned short nCurrValue = 0;
	unsigned short nLastValue = 0;
	unsigned short nAbsDiffValue = 0;
	short nDiffValue = 0;
	unsigned char cOutStage = 0;
	unsigned char cOutChar = 0;
	unsigned char cZeroCounter = 0;
	static unsigned short nEmbTable[XN_MAX_UINT16];
	unsigned short nEmbTableIdx = 0;

	// Determine nMaxValue
	unsigned short nMaxValue = KINECT_MAX_DEPTH;

	// Validate the input/output pointers (to make sure none of them is NULL)
	// XN_VALIDATE_INPUT_PTR(pInput);
	// XN_VALIDATE_INPUT_PTR(pOutput);
	// XN_VALIDATE_INPUT_PTR(pnOutputSize);

	// Create the embedded value translation table...
	pOutput += 2;
	for (int i = 0; i < XN_MAX_UINT16; i++) nEmbTable[i] = 0;
	// xnOSMemSet(&nEmbTable[0], 0, nMaxValue*sizeof(unsigned short));

	while (pInput != pInputEnd)
	{
		nEmbTable[*pInput] = 1;
		pInput++;
	}

	for (unsigned long i = 0; i < XN_MAX_UINT16; i++)
	{
		if (nEmbTable[i] == 1)
		{
			nEmbTable[i] = nEmbTableIdx;
			nEmbTableIdx++;
			*(unsigned short *)pOutput = unsigned short(i);
			pOutput += 2;
		}
	}

	*(unsigned short *)(pOrigOutput) = nEmbTableIdx;

	// Encode the data...
	pInput = pOrigInput;
	nLastValue = nEmbTable[*pInput];
	*(unsigned short *)pOutput = nLastValue;
	//*(unsigned short*)pOutput = XN_PREPARE_VAR16_IN_BUFFER(nLastValue);
	pInput++;
	pOutput += 2;

	// 	for (unsigned long i = 0; i < nEmbTableIdx; i++)
	// 		nEmbTable[i] = XN_PREPARE_VAR16_IN_BUFFER(nEmbTable[i]);

	while (pInput < pInputEnd)
	{
		nCurrValue = nEmbTable[*pInput];
		nDiffValue = (nLastValue - nCurrValue);
		nAbsDiffValue = ((nDiffValue > 0) ? nDiffValue : (-nDiffValue));

		if (nAbsDiffValue <= 6)
		{
			nDiffValue += 6;

			if (cOutStage == 0)
			{
				cOutChar = nDiffValue << 4;
				cOutStage = 1;
			}
			else
			{
				cOutChar += nDiffValue;

				if (cOutChar == 0x66)
				{
					cZeroCounter++;

					if (cZeroCounter == 15)
					{
						*pOutput = 0xEF;
						pOutput++;

						cZeroCounter = 0;
					}
				}
				else
				{
					if (cZeroCounter != 0)
					{
						*pOutput = 0xE0 + cZeroCounter;
						pOutput++;

						cZeroCounter = 0;
					}

					*pOutput = cOutChar;
					pOutput++;
				}
				cOutStage = 0;
			}
		}
		else
		{
			if (cZeroCounter != 0)
			{
				*pOutput = 0xE0 + cZeroCounter;
				pOutput++;

				cZeroCounter = 0;
			}

			if (cOutStage == 0)
			{
				cOutChar = 0xFF;
			}
			else
			{
				cOutChar += 0x0F;
				cOutStage = 0;
			}

			*pOutput = cOutChar;
			pOutput++;

			if (nAbsDiffValue <= 63)
			{
				nDiffValue += 192;

				*pOutput = (unsigned char)nDiffValue;
				pOutput++;
			}
			else
			{
				*(unsigned short *)pOutput = (nCurrValue << 8) + (nCurrValue >> 8);
				// *(unsigned short*)pOutput = XN_PREPARE_VAR16_IN_BUFFER((nCurrValue << 8) + (nCurrValue >> 8));
				pOutput += 2;
			}
		}

		nLastValue = nCurrValue;
		pInput++;
	}

	if (cOutStage != 0)
	{
		*pOutput = cOutChar + 0x0D;
		pOutput++;
	}

	if (cZeroCounter != 0)
	{
		*pOutput = 0xE0 + cZeroCounter;
		pOutput++;
	}

	*pnOutputSize = pOutput - pOrigOutput;

	// All is good...
	//return (XN_STATUS_OK);
	return 0;
}

int DepthCodec::decode(const unsigned char *pInput,
					   const unsigned long nInputSize,
					   unsigned short *pOutput,
					   unsigned long *pnOutputSize)
{
	// Local function variables
	const unsigned char *pInputEnd = pInput + nInputSize;
	unsigned short *pOutputEnd = 0;
	unsigned short *pOrigOutput = pOutput;
	unsigned short nLastFullValue = 0;
	unsigned char cInput = 0;
	unsigned char cZeroCounter = 0;
	char cInData1 = 0;
	char cInData2 = 0;
	unsigned char cInData3 = 0;
	unsigned short *pEmbTable = NULL;
	unsigned short nEmbTableIdx = 0;

	// Validate the input/output pointers (to make sure none of them is NULL)
	// XN_VALIDATE_INPUT_PTR(pInput);
	// XN_VALIDATE_INPUT_PTR(pOutput);
	// XN_VALIDATE_INPUT_PTR(pnOutputSize);
	//
	//	if (nInputSize < sizeof(unsigned short))
	//	{
	//		return (XN_STATUS_BAD_PARAM);
	//}

	nEmbTableIdx = *(unsigned short *)pInput;
	// nEmbTableIdx = XN_PREPARE_VAR16_IN_BUFFER(*(unsigned short*)pInput);
	pInput += 2;
	pEmbTable = (unsigned short *)pInput;
	pInput += nEmbTableIdx * 2;
	//for (unsigned long i = 0; i < nEmbTableIdx; i++) {
	// pEmbTable[i] = XN_PREPARE_VAR16_IN_BUFFER(pEmbTable[i]);
	//}

	pOutputEnd = pOutput + (*pnOutputSize / sizeof(unsigned short));

	// Decode the data...
	nLastFullValue = *(unsigned short *)pInput;
	// nLastFullValue = XN_PREPARE_VAR16_IN_BUFFER(*(unsigned short*)pInput);
	*pOutput = pEmbTable[nLastFullValue];
	pInput += 2;
	pOutput++;

	while (pInput != pInputEnd)
	{
		cInput = *pInput;

		if (cInput < 0xE0)
		{
			cInData1 = cInput >> 4;
			cInData2 = (cInput & 0x0f);

			nLastFullValue -= (cInData1 - 6);
			if (pOutput > pOutputEnd) return -1;
			//XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
			*pOutput = pEmbTable[nLastFullValue];
			pOutput++;

			if (cInData2 != 0x0f)
			{
				if (cInData2 != 0x0d)
				{
					nLastFullValue -= (cInData2 - 6);
					if (pOutput > pOutputEnd) return -1;
					// XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
					*pOutput = pEmbTable[nLastFullValue];
					pOutput++;
				}

				pInput++;
			}
			else
			{
				pInput++;

				cInData3 = *pInput;
				if (cInData3 & 0x80)
				{
					nLastFullValue -= (cInData3 - 192);
					if (pOutput > pOutputEnd)
						return -1;
					//XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
					*pOutput = pEmbTable[nLastFullValue];

					pOutput++;
					pInput++;
				}
				else
				{
					nLastFullValue = cInData3 << 8;
					pInput++;
					nLastFullValue += *pInput;
					if (pOutput > pOutputEnd) return -1;
					//XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
					*pOutput = pEmbTable[nLastFullValue];

					pOutput++;
					pInput++;
				}
			}
		}
		else if (cInput == 0xFF)
		{
			pInput++;

			cInData3 = *pInput;

			if (cInData3 & 0x80)
			{
				nLastFullValue -= (cInData3 - 192);
				if (pOutput > pOutputEnd)
					return -1;
				//XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
				*pOutput = pEmbTable[nLastFullValue];

				pInput++;
				pOutput++;
			}
			else
			{
				nLastFullValue = cInData3 << 8;
				pInput++;
				nLastFullValue += *pInput;
				if (pOutput > pOutputEnd) return -1;
				//XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
				*pOutput = pEmbTable[nLastFullValue];

				pInput++;
				pOutput++;
			}
		}
		else //It must be 0xE?
		{
			cZeroCounter = cInput - 0xE0;

			while (cZeroCounter != 0)
			{
				if (pOutput + 1 > pOutputEnd) return -1;
				//XN_CHECK_OUTPUT_OVERFLOW(pOutput+1, pOutputEnd);
				*pOutput = pEmbTable[nLastFullValue];
				pOutput++;

				*pOutput = pEmbTable[nLastFullValue];
				pOutput++;

				cZeroCounter--;
			}
			pInput++;
		}
	}

	*pnOutputSize = (pOutput - pOrigOutput) * sizeof(unsigned short);

	// All is good...
	//return (XN_STATUS_OK);
	return 0;
}
