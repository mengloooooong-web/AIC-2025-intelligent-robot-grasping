#pragma once

/*
 * File - NotImplementedException.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file NotImplementedException.h
 *
 * \brief Not implamanted exception
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief NotImplementedException
     *
     * Generates an exception, wihch stack trace
     */
    class NotImplementedException : public std::logic_error
    {
        int addrlen;
        char** symbolList;

    public:

        /**
         * \brief NotImplementedException constructor
         *
         * \param str String to be emitted as part of the execption trace
         */
        NotImplementedException(char const* str) throw();

        /**
         * \brief Return explanatory string
         *
         * Allows non-implemented methods to create a stack trace of their invocation.
         *
         * \return pointer to null-terminated string describing the exceptiondescribing the exception
         */
        virtual const char * what() const throw();
    };
}
