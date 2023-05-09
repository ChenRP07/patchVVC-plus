#include "render/render.h"

int main() {
	try {
		std::string   vertex_source   = "/mnt/ssd1/crp/vvc/src/render/vertex.shader";
		std::string   fragment_source = "/mnt/ssd1/crp/vvc/src/render/fragment.shader";
		std::ifstream input_vertex_shader(vertex_source);
		if (input_vertex_shader.fail()) {
			switch (errno) {
				case ENOENT: throw __EXCEPT__(FILE_NOT_EXIST); break;
				case EACCES: throw __EXCEPT__(PERMISSION_DENIED); break;
				default: throw __EXCEPT__(UNEXPECTED_FILE_ERROR); break;
			}
		}
		std::ostringstream vertex_shader_stream;
		vertex_shader_stream << input_vertex_shader.rdbuf();
		std::string vertex_shader_ = vertex_shader_stream.str();

		std::ifstream input_fragment_shader(fragment_source);
		if (input_fragment_shader.fail()) {
			switch (errno) {
				case ENOENT: throw __EXCEPT__(FILE_NOT_EXIST); break;
				case EACCES: throw __EXCEPT__(PERMISSION_DENIED); break;
				default: throw __EXCEPT__(UNEXPECTED_FILE_ERROR); break;
			}
		}
		std::ostringstream fragment_shader_stream;
		fragment_shader_stream << input_fragment_shader.rdbuf();
		std::string fragment_shader_ = fragment_shader_stream.str();

		std::cout << vertex_shader_ << '\n' << fragment_shader_ << '\n';
	}
	catch (const vvc::common::Exception& e) {
		e.Log();
		exit(1);
	}
}
