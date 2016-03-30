////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C), 2015, �Ͼ�����ѧ�������ѧ�빤��ѧԺ, ���ܿ�ѧ�뼼��ϵ
//  FileName:  NJUST_MO_proc.h
//  Author: ������
//  Date:   2015.6.26
//  Description: �Կ�ģ�������ģ���������״̬
//  Modification: 
//          2015.7.2, ������
//  Functions:
//
//           int NJUST_MO_Decode_IP_Data_CMD( const void* pIPData, 
//                                            const int nBytes,
//                                            NJUST_FROM_MO_COMMAND  **pCommand //�����������Կص�����ʱ,ֵΪNULL
//						                    );
//
//           void* NJUST_MO_Encode_STA_IP_Data( NJUST_TO_MO_WORKSTAT *pWorkStat,
//                                              int *nBytes,
//						                      );
//
////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _NJUST_MO_PROC_H_
#define _NJUST_MO_PROC_H_

#include "NJUST_MO_data.h" 

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////////////////////
//
//  �Կ��ṩ��Ŀ��ģ��Ľ��뺯��
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief �ṩ��Ŀ��ģ��Ľ��뺯��
 * @param pIPData {const void*} [in] IP����,Ŀ��ģ��������յ�������
 * @param nBytes {const int} [in] pIPData���ݵ��ֽڸ���
 * @param pCommand {NJUST_FROM_MO_COMMAND **} [out] �����������Կص�����ʱ,ֵΪ��ָ��NULL
 * @return {int}, �ɹ�����0, ����-1
 */
int NJUST_MO_Decode_IP_Data_CMD( const void* pIPData,
							     const int nBytes,
                                 NJUST_FROM_MO_COMMAND   **pCommand //������������������ʱ,ֵΪNULL
						       );
////////////////////////////////////////////////////////////////////////////////////////////////
//
//  �ṩ��Ŀ��ģ��ı��뺯��
//
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief �ṩ��Ŀ��ģ��ı��뺯��
 * @param pWorkStat {NJUST_TO_MO_WORKSTAT *} [in] ָ��״̬����; 
 * @param nBytes {int *} [out] �������ݵ��ֽڸ���
 * @return {void *}, �ɹ���������ָ��, ���򷵻�NULL
 */
void* NJUST_MO_Encode_STA_IP_Data( NJUST_TO_MO_WORKSTAT *pWorkStat,
                                   int *nBytes
					             );

#ifdef __cplusplus
}
#endif

#endif