/********************************************************************************
 *                                                                              *
 * This file is part of IfcOpenShell.                                           *
 *                                                                              *
 * IfcOpenShell is free software: you can redistribute it and/or modify         *
 * it under the terms of the Lesser GNU General Public License as published by  *
 * the Free Software Foundation, either version 3.0 of the License, or          *
 * (at your option) any later version.                                          *
 *                                                                              *
 * IfcOpenShell is distributed in the hope that it will be useful,              *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of               *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 *
 * Lesser GNU General Public License for more details.                          *
 *                                                                              *
 * You should have received a copy of the Lesser GNU General Public License     *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.         *
 *                                                                              *
 ********************************************************************************/

#ifndef STEPSERIALIZER_H
#define STEPSERIALIZER_H

#include <stdio.h>
#if defined(_MSC_VER) && defined(_UNICODE)
#include <stdlib.h>
#endif

#include <STEPControl_Writer.hxx>
#include <Interface_Static.hxx>

#include "../ifcgeom/IfcGeomIterator.h"

#include "../ifcconvert/OpenCascadeBasedSerializer.h"

class StepSerializer : public OpenCascadeBasedSerializer
{
private:
	STEPControl_Writer writer;	
public:
#if defined(_MSC_VER) && defined(_UNICODE)
	explicit StepSerializer(const std::wstring& out_filename) 
#else
	explicit StepSerializer(const std::string& out_filename) 
#endif
		: OpenCascadeBasedSerializer(out_filename) 
	{}
	virtual ~StepSerializer() {}
	void writeShape(const TopoDS_Shape& shape) {
		std::stringstream ss;
		std::streambuf *sb = std::cout.rdbuf(ss.rdbuf());
		writer.Transfer(shape, STEPControl_AsIs);
		std::cout.rdbuf(sb);
	}
	void finalize() {
		std::stringstream ss;
		std::streambuf *sb = std::cout.rdbuf(ss.rdbuf());
#if defined(_MSC_VER) && defined(_UNICODE)
		// hack hack hack
		char* tmp_filename = tmpnam(0);
		size_t tmp_filename_size = strlen(tmp_filename) + 1;
		wchar_t* tmp_filename_wide = new wchar_t[tmp_filename_size];
		writer.Write(tmp_filename);
		size_t out;
		mbstowcs_s(&out, tmp_filename_wide, tmp_filename_size, tmp_filename, tmp_filename_size - 1);
		_wrename(tmp_filename_wide, out_filename.c_str());
		delete[] tmp_filename_wide;
#else
		writer.Write(out_filename.c_str());
#endif				
		std::cout.rdbuf(sb);
	}
	void setUnitNameAndMagnitude(const std::string& /*name*/, float magnitude) {
		const char* symbol = getSymbolForUnitMagnitude(magnitude);
		if (symbol) {
			Interface_Static::SetCVal("xstep.cascade.unit", symbol);
			Interface_Static::SetCVal("write.step.unit", symbol);
		}
	}
};

#endif