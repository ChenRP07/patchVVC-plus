/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/03/17 14:51
 * Last Modified : 2023/04/14 10:33
 *
 */

#include "octree/octree.h"

namespace vvc {
namespace octree {

	void SaveTreeCore(pcl::PointXYZ _center, pcl::PointXYZ _range, int _height, std::shared_ptr<std::vector<uint8_t>> _p) {
		float     data[6] = {_center.x, _center.y, _center.z, _range.x, _range.y, _range.z};
		uint32_t* temp    = nullptr;
		for (auto i : data) {
			temp = reinterpret_cast<uint32_t*>(&i);
			_p->emplace_back((*temp) >> 24);
			_p->emplace_back((*temp) >> 16);
			_p->emplace_back((*temp) >> 8);
			_p->emplace_back((*temp) >> 0);
		}
		_p->emplace_back(static_cast<uint8_t>(_height));
	}

	void LoadTreeCore(pcl::PointXYZ& _center, pcl::PointXYZ& _range, int& _height, uint8_t (&_p)[25]) {
		float data[6] = {};

		for (int i = 0; i < 6; ++i) {
			int d{0};
			for (int j = 0; j < 4; ++j) {
				d <<= 8;
				d |= _p[i * 4 + j];
			}
			float* t = reinterpret_cast<float*>(&d);
			data[i]  = *t;
		}
		_center.x = data[0], _center.y = data[1], _center.z = data[2];
		_range.x = data[3], _range.y = data[4], _range.z = data[5];
		_height = _p[24];
	}

	OctreeBase::OctreeBase() : tree_range_{0.0f, 0.0f, 0.0f}, tree_height_{0}, params_{nullptr}, tree_center_{} {}

	void OctreeBase::SetParams(common::PVVCParam_t::Ptr _param) {
		try {
			if (!_param) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			this->params_ = _param;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void OctreeNode_t::HierarchicalTransform() {
		common::ColorYUV H[8];
		/* X/Y/Z merge */
		for (int i = 7; i > 0; --i) {
			std::pair<common::ColorYUV, common::ColorYUV> g(this->raht[NodeWeight[i][0]], this->raht[NodeWeight[i][1]]), res;
			std::pair<int, int>                           w(this->weight[NodeWeight[i][0]], this->weight[NodeWeight[i][1]]);
			/* Haar wavelet transform */
			res = HaarTransform(w, g);
			/* Record g and h */
			this->raht[i] = res.first;
			H[i]          = res.second;
		}
		/* Save last g and all h */
		this->raht[0] = this->raht[1];
		for (int i = 1; i <= 7; ++i) {
			this->raht[i] = H[i];
		}
	}

	void OctreeNode_t::InvertHierarchicalTransform() {
		common::ColorYUV H[8];
		for (int i = 0; i < 8; ++i) {
			H[i] = this->raht[i];
		}
		/* g_DC */
		this->raht[1] = H[0];
		/* Invert X/Y/Z merge */
		for (int i = 1; i < 8; ++i) {
			std::pair<common::ColorYUV, common::ColorYUV> g(this->raht[i], H[i]), res;
			std::pair<int, int>                           w(this->weight[NodeWeight[i][0]], this->weight[NodeWeight[i][1]]);
			/* Inver haar wavelet transform */
			res                          = InvertHaarTransform(w, g);
			this->raht[NodeWeight[i][0]] = res.first;
			this->raht[NodeWeight[i][1]] = res.second;
		}
	}

	pcl::PointXYZ SubSpaceCenter(pcl::PointXYZ _center, pcl::PointXYZ _range, int _pos) {
		try {
			switch (_pos) {
				case 0: return pcl::PointXYZ(_center.x + _range.x / 2.0f, _center.y + _range.y / 2.0f, _center.z + _range.z / 2.0f);
				case 1: return pcl::PointXYZ(_center.x + _range.x / 2.0f, _center.y + _range.y / 2.0f, _center.z - _range.z / 2.0f);
				case 2: return pcl::PointXYZ(_center.x + _range.x / 2.0f, _center.y - _range.y / 2.0f, _center.z + _range.z / 2.0f);
				case 3: return pcl::PointXYZ(_center.x + _range.x / 2.0f, _center.y - _range.y / 2.0f, _center.z - _range.z / 2.0f);
				case 4: return pcl::PointXYZ(_center.x - _range.x / 2.0f, _center.y + _range.y / 2.0f, _center.z + _range.z / 2.0f);
				case 5: return pcl::PointXYZ(_center.x - _range.x / 2.0f, _center.y + _range.y / 2.0f, _center.z - _range.z / 2.0f);
				case 6: return pcl::PointXYZ(_center.x - _range.x / 2.0f, _center.y - _range.y / 2.0f, _center.z + _range.z / 2.0f);
				case 7: return pcl::PointXYZ(_center.x - _range.x / 2.0f, _center.y - _range.y / 2.0f, _center.z - _range.z / 2.0f);
				default: throw __EXCEPT__(ERROR_OCCURED);
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	bool CheckSubSpace(std::vector<std::vector<std::vector<int>>>& _subspaces) {
		/* Check eight subspaces */
		for (auto& i : _subspaces) {
			/* Count how many patch is not empty */
			int count = 0;
			for (auto& j : i) {
				count += j.empty() ? 0 : 1;
			}
			/* All patches are empty or not empty */
			if (count == 0 || count == i.size()) {
				continue;
			}
			/* Some but not all patches are empty */
			else {
				return false;
			}
		}
		return true;
	}

	bool CheckSubSpace(std::vector<std::vector<int>>& _space) {
		/* Count how many patch is not empty */
		int count = 0;
		for (auto& j : _space) {
			count += j.empty() ? 0 : 1;
		}
		/* All patches are empty or not empty */
		if (count == 0 || count == _space.size()) {
			return true;
		}
		return false;
	}

	bool CheckSpaceEmpty(std::vector<std::vector<int>>& _space) {
		for (auto& i : _space) {
			if (!i.empty()) {
				return false;
			}
		}
		return true;
	}

	std::pair<common::ColorYUV, common::ColorYUV> HaarTransform(std::pair<int, int>& _w, std::pair<common::ColorYUV, common::ColorYUV>& _g) {
		common::ColorYUV G, H;
		if (_w.first == 0 && _w.second == 0) {
			return std::make_pair(G, H);
		}

		/*
		 *     [ √w0 √w1 ]
		 *     [-√w1 √w0 ]
		 * T = -----------
		 *     √(w0 + w1)
		 *
		 * */
		float base    = std::sqrt(static_cast<float>(_w.first + _w.second));
		float T[2][2] = {{std::sqrt(static_cast<float>(_w.first)) / base, std::sqrt(static_cast<float>(_w.second)) / base},
		                 {-std::sqrt(static_cast<float>(_w.second)) / base, std::sqrt(static_cast<float>(_w.first)) / base}};

		G = _g.first * T[0][0] + _g.second * T[0][1];
		H = _g.first * T[1][0] + _g.second * T[1][1];
		return std::make_pair(G, H);
	}

	std::pair<common::ColorYUV, common::ColorYUV> InvertHaarTransform(std::pair<int, int>& _w, std::pair<common::ColorYUV, common::ColorYUV>& _g) {
		common::ColorYUV G, H;
		if (_w.first == 0 && _w.second == 0) {
			return std::make_pair(G, H);
		}

		/*
		 *        [ √w0 -√w1]
		 *        [ √w1 √w0 ]
		 * T^-1 = -----------
		 *        √(w0 + w1)
		 *
		 * */
		float base    = std::sqrt(static_cast<float>(_w.first + _w.second));
		float T[2][2] = {{std::sqrt(static_cast<float>(_w.first)) / base, -std::sqrt(static_cast<float>(_w.second)) / base},
		                 {std::sqrt(static_cast<float>(_w.second)) / base, std::sqrt(static_cast<float>(_w.first)) / base}};

		G = _g.first * T[0][0] + _g.second * T[0][1];
		H = _g.first * T[1][0] + _g.second * T[1][1];
		return std::make_pair(G, H);
	}
}  // namespace octree
}  // namespace vvc
