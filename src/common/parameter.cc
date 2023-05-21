/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Implementation of parameters.
 * Create Time   : 2022/12/12 16:38
 * Last Modified : 2022/12/27 14:06
 *
 */

#include "common/parameter.h"

namespace vvc {
namespace common {
	PVVCParam_t::Ptr SetDefaultParams() {
		try {
			PVVCParam_t p;
			/* 0Bxxxxxxx1 brief 0Bxxxxxx1x normal 0Bxxxxx1xx complete 0Bxxxx1xxx total 0Bxxx1xxxx GoP 0Bxx1xxxxx patch */
			p.log_level = 0xff;

			p.segment.type      = common::DENSE_SEGMENT;
			p.segment.num       = 2048;
			p.segment.nn        = 10;
			p.segment.block_num = 8.0f;

			p.thread_num = 30;
			p.zstd_level = 22;

			p.icp.centroid_alignment = true;
			p.icp.correspondence_ths = 100.0f;
			p.icp.iteration_ths      = 100;
			p.icp.mse_ths            = 0.01f;
			p.icp.transformation_ths = 1e-6;
			p.icp.radius_search_ths  = 10.0f;
			p.icp.type               = SIMPLE_ICP;

			p.octree.resolution = 1.0f;

			p.patch.fitting_ths        = 10.0f;
			p.patch.max_iter           = 100;
			p.patch.split_method       = DIRECT_CLUSTERING;
			p.patch.clustering_ths     = 1.0f;
			p.patch.interpolation_num  = 10;
			p.patch.clustering_err_ths = 0.1f;

			return std::make_shared<const PVVCParam_t>(p);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	PVVCParam_t::Ptr CopyParams(PVVCParam_t::Ptr _ptr) {
		try {
			if (!_ptr) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			PVVCParam_t p;

			p.log_level  = _ptr->log_level;
			p.thread_num = _ptr->thread_num;
			p.zstd_level = _ptr->zstd_level;

			p.segment.type      = _ptr->segment.type;
			p.segment.num       = _ptr->segment.num;
			p.segment.nn        = _ptr->segment.nn;
			p.segment.block_num = _ptr->segment.block_num;

			p.icp.centroid_alignment = _ptr->icp.centroid_alignment;
			p.icp.correspondence_ths = _ptr->icp.correspondence_ths;
			p.icp.iteration_ths      = _ptr->icp.iteration_ths;
			p.icp.mse_ths            = _ptr->icp.mse_ths;
			p.icp.transformation_ths = _ptr->icp.transformation_ths;
			p.icp.radius_search_ths  = _ptr->icp.radius_search_ths;
			p.icp.type               = SIMPLE_ICP;

			return std::make_shared<const PVVCParam_t>(p);
		}
		catch (const Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	ParameterLoader::ParameterLoader() : cfg_name_{"patchvvc.cfg"}, cfg_{} {}

	ParameterLoader::ParameterLoader(const std::string& _name) : cfg_name_{_name}, cfg_{} {}

	PVVCParam_t::Ptr ParameterLoader::GetPVVCParam() {
		try {
			PVVCParam_t p;
			try {
				this->cfg_.readFile(this->cfg_name_.c_str());
			}
			catch (const libconfig::FileIOException& e) {
				throw __EXCEPT__(UNEXPECTED_FILE_ERROR);
			}
			catch (const libconfig::ParseException& e) {
				std::cout << __REDT__([Error]) << " " << e.getError() << '\n';
				throw __EXCEPT__(BAD_PARAMETERS);
			}

			uint32_t temp_uint;
			if (!this->cfg_.lookupValue("log_level", temp_uint)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(log_level will be set to brief since it is not in cfg.) << '\n';
				p.log_level = 0x01;
			}
			else {
				p.log_level = temp_uint;
			}

			if (!this->cfg_.lookupValue("check_point", temp_uint)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(check_point will be set to none since it is not in cfg.) << '\n';
				p.check_point = 0x00;
			}
			else {
				p.check_point = temp_uint;
			}

			if (!this->cfg_.lookupValue("thread_num", p.thread_num)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(thread_num will be set to 30 since it is not in cfg.) << '\n';
				p.thread_num = 30;
			}

			if (!this->cfg_.lookupValue("zstd_level", p.zstd_level)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(zstd_level will be set to 22 since it is not in cfg.) << '\n';
				p.zstd_level = 22;
			}

			if (!this->cfg_.lookupValue("max_keyframe", p.max_keyframe)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(max_keyframe will be set to 150 since it is not in cfg.) << '\n';
				p.max_keyframe = 150;
			}

			if (!this->cfg_.lookupValue("start_timestamp", p.start_timestamp)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(start_timestamp will be set to 0 since it is not in cfg.) << '\n';
				p.start_timestamp = 0;
			}

			if (!this->cfg_.lookupValue("time_interval", p.time_interval)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(time_interval will be set to 1 since it is not in cfg.) << '\n';
				p.time_interval = 1;
			}

			if (!this->cfg_.lookupValue("frames", p.frames)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(frames will be set to 300 since it is not in cfg.) << '\n';
				p.frames = 300;
			}

			if (!this->cfg_.lookupValue("io.sequence_name", p.io.sequence_name)) {
				std::cout << __REDT__([Error]) << " Missing key parameter sequence_name.\n";
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			if (!this->cfg_.lookupValue("io.source_file", p.io.source_file)) {
				std::cout << __REDT__([Error]) << " Missing key parameter source_file.\n";
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			if (!this->cfg_.lookupValue("io.segment_file", p.io.segment_file)) {
				if (p.check_point & 0x01) {
					std::cout << __REDT__([Error]) << " Missing key parameter segment_file when check_point is not none.\n";
					throw __EXCEPT__(EMPTY_PARAMS);
				}
			}
			
            if (!this->cfg_.lookupValue("io.deform_file", p.io.deform_file)) {
				std::cout << __REDT__([Error]) << " Missing key parameter result_file.\n";
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			if (!this->cfg_.lookupValue("io.result_file", p.io.result_file)) {
				std::cout << __REDT__([Error]) << " Missing key parameter result_file.\n";
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			std::string temp_s;

			if (!this->cfg_.lookupValue("segment.num", p.segment.num)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(segment.num will be set to 2048 since it is not in cfg.) << '\n';
				p.segment.num = 2048;
			}

			if (!this->cfg_.lookupValue("segment.type", temp_s)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(segment.type will by set to DENSE_SEGMENT since it is no in cfg.) << '\n';
				p.segment.type = DENSE_SEGMENT;
			}
			else {
				if (temp_s == "dense_segment") {
					p.segment.type = DENSE_SEGMENT;
				}
				else {
					throw __EXCEPT__(BAD_PARAMETERS);
				}
			}

			if (!this->cfg_.lookupValue("segment.nn", p.segment.nn)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(segment.nn will be set to 10 since it is not in cfg.) << '\n';
				p.segment.num = 10;
			}

			if (!this->cfg_.lookupValue("segment.block_num", p.segment.block_num)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(segment.block_num will be set to 8.0f since it is not in cfg.) << '\n';
				p.segment.block_num = 8.0f;
			}

			if (!this->cfg_.lookupValue("icp.correspondence_ths", p.icp.correspondence_ths)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(icp.correspondence_ths will be set to 100.0f since it is not in cfg.) << '\n';
				p.icp.correspondence_ths = 100.0f;
			}

			if (!this->cfg_.lookupValue("icp.iteration_ths", p.icp.iteration_ths)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(icp.iteration_ths will be set to 100 since it is not in cfg.) << '\n';
				p.icp.iteration_ths = 100;
			}

			if (!this->cfg_.lookupValue("icp.mse_ths", p.icp.mse_ths)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(icp.mse_ths will be set to 0.01f since it is not in cfg.) << '\n';
				p.icp.mse_ths = 0.01f;
			}

			if (!this->cfg_.lookupValue("icp.transformation_ths", p.icp.transformation_ths)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(icp.transformation_ths will be set to 1e-6 since it is not in cfg.) << '\n';
				p.icp.transformation_ths = 1e-6;
			}

			if (!this->cfg_.lookupValue("icp.radius_search_ths", p.icp.radius_search_ths)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(icp.radius_search_ths will be set to 10.0f since it is not in cfg.) << '\n';
				p.icp.radius_search_ths = 10.0f;
			}

			if (!this->cfg_.lookupValue("icp.centroid_alignment", p.icp.centroid_alignment)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(icp.centroid_alignment will be set to true since it is not in cfg.) << '\n';
				p.icp.centroid_alignment = true;
			}

			if (!this->cfg_.lookupValue("icp.type", temp_s)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(icp.type will be set to SIMPLE_ICP since it is not in cfg.) << '\n';
				p.icp.type = SIMPLE_ICP;
			}
			else {
				if (temp_s == "simple_icp") {
					p.icp.type = SIMPLE_ICP;
				}
				else if (temp_s == "lm_icp") {
					p.icp.type = LM_ICP;
				}
				else if (temp_s == "normal_icp") {
					p.icp.type = NORMAL_ICP;
				}
				else if (temp_s == "general_icp") {
					p.icp.type = GENERAL_ICP;
				}
				else {
					throw __EXCEPT__(BAD_PARAMETERS);
				}
			}

			if (!this->cfg_.lookupValue("slice.qp_i", temp_uint)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(slice.qp_i will be set to 10 since it is not in cfg.) << '\n';
				p.slice.qp_i = 10;
			}
			else {
				p.slice.qp_i = temp_uint;
			}

			if (!this->cfg_.lookupValue("slice.qp_p", temp_uint)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(slice.qp_p will be set to 30 since it is not in cfg.) << '\n';
				p.slice.qp_p = 30;
			}
			else {
				p.slice.qp_p = temp_uint;
			}

			if (!this->cfg_.lookupValue("octree.resolution", p.octree.resolution)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(octree.resolution will be set to 1.0f since it is not in cfg.) << '\n';
				p.octree.resolution = 1.0f;
			}

			if (!this->cfg_.lookupValue("patch.fitting_ths", p.patch.fitting_ths)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(patch.fitting_ths will be set to 10.0f since it is not in cfg.) << '\n';
				p.patch.fitting_ths = 10.0f;
			}

			if (!this->cfg_.lookupValue("patch.max_iter", p.patch.max_iter)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(patch.max_iter will be set to 100 since it is not in cfg.) << '\n';
				p.patch.max_iter = 100;
			}

			if (!this->cfg_.lookupValue("patch.clustering_ths", p.patch.clustering_ths)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(path.clustering_ths will be set to 1.0f since it is not in cfg.) << '\n';
				p.patch.clustering_ths = 1.0f;
			}

			if (!this->cfg_.lookupValue("patch.clustering_err_ths", p.patch.clustering_err_ths)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(path.clustering_err_ths will be set to 0.1f since it is not in cfg.) << '\n';
				p.patch.clustering_err_ths = 0.1f;
			}

			if (!this->cfg_.lookupValue("patch.interpolation_num", p.patch.interpolation_num)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(path.interpolation_num will be set to 10 since it is not in cfg.) << '\n';
				p.patch.interpolation_num = 10;
			}

			if (!this->cfg_.lookupValue("patch.split_method", temp_s)) {
				std::cout << __YELLOWT__([Warning]) << ' ' << __AZURET__(path.split_method will be set to DIRECT_CLUSTERING since it is not in cfg.) << '\n';
				p.patch.split_method = DIRECT_CLUSTERING;
			}
			else {
				if (temp_s == "direct_clustering") {
					p.patch.split_method = DIRECT_CLUSTERING;
				}
				else if (temp_s == "planar_bisection") {
					p.patch.split_method = PLANAR_BISECTION;
				}
				else if (temp_s == "partial_clustering") {
					p.patch.split_method = PARTIAL_CLUSTERING;
				}
				else {
					throw __EXCEPT__(BAD_PARAMETERS);
				}
			}

			return std::make_shared<const PVVCParam_t>(p);
		}
		catch (const Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	// clang-format off
	void PVVCParam_t::Log() const {
		PVVCLog_Mutex.lock();
        std::cout << __AZURET__(================================================) << '\n';
        std::cout << "Parameters of patchVVC : \n";
        std::cout << "Log level : ";
        if (this->log_level & 0x08) std::cout << "complete\n";
        else if (this->log_level & 0x04) std::cout << "normal\n";
        else if (this->log_level & 0x02) std::cout << "brief\n";
        else std::cout << "quiet\n";

        std::cout << "Check point : ";
        if (this->check_point & 0x10) std::cout << "slices ";
        if (this->check_point & 0x08) std::cout << "fitting_cloud ";
        if (this->check_point & 0x04) std::cout << "all_patches ";
        if (this->check_point & 0x02) std::cout << "intra_patches ";
        std::cout << "--\n";

        printf("Launch threads : %d\n", this->thread_num);
        printf("Zstd level : %d\n", this->zstd_level);
        printf("Max interval of keyframe : %d\n", this->max_keyframe);
        printf("Start frame timestamp : %d\n", this->start_timestamp);
        printf("Frames timestamp interval : %d\n", this->time_interval);
        printf("Total frames : %d\n", this->frames);
        printf("Sequence name : %s\n", this->io.sequence_name.c_str());
        printf("Source file path : %s\n", this->io.source_file.c_str());
        printf("Buffer file path : %s\n", this->io.segment_file.c_str());
        printf("Result file path : %s\n", this->io.result_file.c_str());
        printf("Avg patch point number : %d\n", this->segment.num);
        printf("KNN neighbors : %d\n", this->segment.nn);
        printf("Segment block number : %d\n", static_cast<int>(this->segment.block_num));        
        printf("Segment method : ");
        switch (this->segment.type) {
            default: printf("--\n"); break;
            case SEGMENT_TYPE::DENSE_SEGMENT: printf("dense segment\n"); break;
        }

        printf("Max ICP correspondence distance : %.2f\n", this->icp.correspondence_ths);
        printf("Max ICP iteration : %d\n", this->icp.iteration_ths);
        printf("Max ICP MSE : %.2f\n", this->icp.mse_ths);
        printf("Max ICP transformation distance : %f\n", this->icp.transformation_ths);
        printf("Max ICP radius search distance : %.2f\n",this->icp.radius_search_ths);
        printf("ICP type : ");
        switch (this->icp.type) {
            default: printf("--\n"); break;
            case ICP_TYPE::SIMPLE_ICP: printf("simple\n"); break;
            case ICP_TYPE::LM_ICP: printf("LM\n"); break;
            case ICP_TYPE::NORMAL_ICP: printf("normal\n"); break;
            case ICP_TYPE::GENERAL_ICP: printf("general\n"); break;
        }
        printf("Centroid alignment : %s\n", this->icp.centroid_alignment ? "Yes" : "No");
        printf("QP for intra slice : %d\n", this->slice.qp_i);
        printf("QP for predict slice: %d\n", this->slice.qp_p);
        printf("Octree min resolution : %.2f\n", this->octree.resolution);
        printf("Max fitting MSE : %.2f\n", this->patch.fitting_ths);
        printf("Min clustering resolution : %.2f\n", this->patch.clustering_ths);
        printf("Color interpolation neighbors : %d\n", this->patch.interpolation_num);
        printf("Max clustering iteration : %d\n", this->patch.max_iter);
        printf("Split method in patch fitting : ");
        switch (this->patch.split_method) {
            default: printf("--\n"); break;
            case SPLIT_TYPE::DIRECT_CLUSTERING: printf("direct clustering\n"); break;
            case SPLIT_TYPE::PLANAR_BISECTION: printf("planar bisection\n"); break;
            case SPLIT_TYPE::PARTIAL_CLUSTERING: printf("partial clustering\n"); break;
        }
        std::cout << __AZURET__(================================================) << '\n';
	    PVVCLog_Mutex.unlock();
	}
	// clang-format on
}  // namespace common
}  // namespace vvc
