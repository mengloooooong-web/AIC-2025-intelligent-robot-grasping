/*
* File - Version.h
*
* This file is part of the Inuitive SDK
*
* Copyright (C) 2014 All rights reserved to Inuitive
*
*/

#define INUROS_VERSION_1  2
#define INUROS_VERSION_2  10
#define INUROS_VERSION_3  1
#define INUROS_VERSION_4  00

#define INUROS_VERSION_1_STR  "2"
#define INUROS_VERSION_2_STR  "10"
#define INUROS_VERSION_3_STR  "1"
#define INUROS_VERSION_4_STR  "00"

#define INUROS_VERSION_STR "2.10.1.00"

#define INUROS_PRODUCT_1  INUROS_VERSION_1
#define INUROS_PRODUCT_2  INUROS_VERSION_2
#define INUROS_PRODUCT_3  INUROS_VERSION_3
#define INUROS_PRODUCT_4  INUROS_VERSION_4

#define INUROS_PRODUCT_STR INUROS_VERSION_STR

static const unsigned int INUROS_VERSION_NUMBER(10000000 * INUROS_VERSION_1 + 100000 * INUROS_VERSION_2 + 1000 * INUROS_VERSION_3 + INUROS_VERSION_4);

static const char * const version = "@(#)Version: " INUROS_VERSION_STR ;
