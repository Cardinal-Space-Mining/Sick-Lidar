#pragma once

#include <cmath>
#include <cstdint>
#include <type_traits>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>


/** Static utilities used in AccumulatorGrid */
class AccumulatorGridBase_ {
protected:
	template<typename Acc_t, typename Stat_t>
	struct CellBase_ {
		using Acc_T = Acc_t;
		using Stat_T = Stat_t;

		inline static constexpr bool
			IsDense = std::is_same<Acc_T, Stat_T>::value;
	};

public:
	template<typename Acc_t, typename Stat_t = float>
	struct GridCell_ : CellBase_<Acc_t, Stat_t> {
		using typename CellBase_<Acc_t, Stat_t>::Acc_T;
		using typename CellBase_<Acc_t, Stat_t>::Stat_T;
		Acc_T val;
		union {
			struct { Stat_T min_z, max_z, avg_z; };
			Stat_T stat[3];
		};
	};
	template<typename Data_t>
	struct GridCell_Aligned_ : CellBase_<Data_t, Data_t> {
		using Data_T = Data_t;
		union {
			struct {
				Data_T val;
				union {
					struct { Data_T min_z, max_z, avg_z; };
					Data_T stat[3];
				};
			};
			Data_T data[4];
		};
	};

	/** If datatypes are the same, use a "dense" allocation, otherwise use standard layout */
	template<typename Acc_t, typename Stat_t>
	using GridCell = typename std::conditional<
		CellBase_<Acc_t, Stat_t>::IsDense,
			GridCell_Aligned_<Acc_t>,
			GridCell_<Acc_t, Stat_t>
		>::type;


public:
	/** Align a point to a box grid of the given resolution and offset origin. Result may be negative if lower than current offset. */
	template<typename IntT = int, typename FloatT = float>
	inline static Eigen::Vector2<IntT> gridAlign(FloatT x, FloatT y, const Eigen::Vector2<FloatT>& off, FloatT res) {
		return Eigen::Vector2<IntT>{
			static_cast<IntT>( std::floor((x - off.x()) / res) ),	// always floor since grid cells are indexed by their "bottom left" corner's raw position
			static_cast<IntT>( std::floor((y - off.y()) / res) )
		};
	}
	template<typename IntT = int, typename FloatT = float>
	inline static Eigen::Vector2<IntT> gridAlign(const Eigen::Vector4<FloatT>& pt, const Eigen::Vector2<FloatT>& off, FloatT res) {
		return gridAlign<IntT, FloatT>(pt.x(), pt.y(), off, res);
	}

	/** Get a raw buffer idx from a 2d index and buffer size (Row~X, Col~Y order) */
	template<typename IntT = int>
	inline static int64_t gridIdx(const IntT x, const IntT y, const Eigen::Vector2<IntT>& size) {
		return static_cast<int64_t>(x) * size.y() + y;	// rows along x-axis, cols along y-axis, thus (x, y) --> x * #cols + y
	}
	template<typename IntT = int>
	inline static int64_t gridIdx(const Eigen::Vector2<IntT>& loc, const Eigen::Vector2<IntT>& size) {
		return gridIdx<IntT>(loc.x(), loc.y(), size);
	}
	template<typename IntT = int>
	inline static Eigen::Vector2<IntT> gridLoc(size_t idx, const Eigen::Vector2<IntT>& size) {
		return Eigen::Vector2<IntT>{
			static_cast<IntT>(idx / size.y()),
			static_cast<IntT>(idx % size.y())
		};
	}


};

template<typename Acc_t = float, typename Int_t = int, typename Float_t = float>
class AccumulatorGrid : public AccumulatorGridBase_ {
	static_assert(std::is_integral_v<Int_t>, "");
	static_assert(std::is_floating_point_v<Float_t>, "");
public:
	using Accumulation_T = Acc_t;
	using IntT = Int_t;
	using FloatT = Float_t;
	using Cell_T = GridCell<Accumulation_T, FloatT>;
	using This_T = AccumulatorGrid<Accumulation_T>;
	using Base_T = AccumulatorGridBase_;

	inline static constexpr bool
		Cell_IsDense = Cell_T::IsDense;
	inline static constexpr size_t
		Cell_Size = sizeof(Cell_T),
		Max_Alloc = (1ULL << 30) / Cell_Size;		// 1 << 30 ~ 1bn --> limit to ~1 gigabyte

	template<int64_t val>
	inline static constexpr Accumulation_T AccVal() { return static_cast<Accumulation_T>(val); }

public:
	AccumulatorGrid() {}
	~AccumulatorGrid() {
		if (this->grid) delete[] this->grid;
	}


	void reset(float grid_res = 1.f, const Eigen::Vector2<FloatT> grid_origin = Eigen::Vector2<FloatT>::Zero()) {
		if (this->grid) {
			delete[] this->grid;
			this->grid = nullptr;
		}
		map_size = Eigen::Vector2<IntT>::Zero();
		max_weight = AccVal<0>();
		resolution = grid_res <= 0.f ? 1.f : grid_res;
		map_origin = grid_origin;
	}

	inline const Eigen::Vector2<IntT>& size() const {
		return this->map_size;
	}
	inline const int64_t area() const {
		return static_cast<int64_t>(this->map_size.x()) * this->map_size.y();
	}
	inline typename std::conditional<(sizeof(Accumulation_T) > 8),
		const Accumulation_T&, const Accumulation_T
	>::type max() const {
		return this->max_weight;
	}
	inline const Eigen::Vector2<FloatT>& origin() const {
		return this->map_origin;
	}
	inline const FloatT gridRes() const {
		return this->resolution;
	}
	inline const Cell_T* gridData() const {
		return this->grid;
	}

	inline const Eigen::Vector2<IntT> boundingCell(const FloatT x, const FloatT y) const {
		return Base_T::gridAlign<IntT, FloatT>(x, y, this->map_origin, this->resolution);
	}
	inline const int64_t cellIdxOf(const FloatT x, const FloatT y) const {
		return Base_T::gridIdx<IntT>(this->boundingCell(x, y), this->map_size);
	}


	/** Returns false if an invalid realloc was skipped */
	bool resizeToBounds(const Eigen::Vector2<FloatT>& min, const Eigen::Vector2<FloatT>& max) {

		static const Eigen::Vector2<IntT>
			_zero = Eigen::Vector2<IntT>::Zero();
		const Eigen::Vector2<IntT>
			_min = this->boundingCell(min.x(), min.y()),	// grid cell locations containing min and max, aligned with current offsets
			_max = this->boundingCell(max.x(), max.y());

		if (_min.cwiseLess(_zero).any() || _max.cwiseGreater(this->map_size).any()) {
			const Eigen::Vector2<IntT>
				_low = _min.cwiseMin(_zero),		// new high and low bounds for the map
				_high = _max.cwiseMax(this->map_size),
				_size = _high - _low;				// new map size

			const int64_t _area = static_cast<int64_t>(_size.x()) * _size.y();
			if (_area > This_T::Max_Alloc || _area < 0) return false;		// less than a gigabyte of allocated buffer is ideal

			Cell_T* _grid = new Cell_T[_area];
			memset(_grid, 0x00, _area * This_T::Cell_Size);	// :O don't forget this otherwise the map will start with all garbage data

			const Eigen::Vector2<IntT> _diff = _zero - _low;	// by how many grid cells did the origin shift
			if (this->grid) {
				for (int r = 0; r < this->map_size.x(); r++) {		// for each row in existing...
					memcpy(									// remap to new buffer
						_grid + ((r + _diff.x()) * _size.y() + _diff.y()),	// (row + offset rows) * new row size + offset cols
						this->grid + (r * this->map_size.y()),
						this->map_size.y() * This_T::Cell_Size
					);
				}
				delete[] this->grid;
			}
			this->grid = _grid;
			this->map_size = _size;
			this->map_origin -= (_diff.template cast<FloatT>() * this->resolution);
		}
		return true;

	}

	template<typename PointT>
	void insertPoints(const pcl::PointCloud<PointT>& cloud, const pcl::Indices& selection) {

		const bool _use_selection = !selection.empty();
		Eigen::Vector4f _min, _max;
		if (_use_selection) {
			pcl::getMinMax3D<PointT>(cloud, selection, _min, _max);
		} else {
			pcl::getMinMax3D<PointT>(cloud, _min, _max);
		}
		if (this->resizeToBounds(
				Eigen::Vector2<FloatT>{ static_cast<FloatT>(_min.x()), static_cast<FloatT>(_min.y()) },
				Eigen::Vector2<FloatT>{ static_cast<FloatT>(_max.x()), static_cast<FloatT>(_min.y()) } )
		) {
			if (_use_selection) {
				for (const pcl::index_t idx : selection) {
					this->insert<PointT>(cloud.points[idx]);
				}
			} else {
				for (const PointT& pt : cloud.points) {
					this->insert<PointT>(pt);
				}
			}
		}

	}

protected:
	/** returns false if accumulation failed (invalid index) */
	template<typename PointT>
	bool insert(const PointT& pt) {
		const int64_t i = this->cellIdxOf(static_cast<FloatT>(pt.x), static_cast<FloatT>(pt.y));
#ifndef SKIP_MAP_BOUND_CHECKS
		if (i >= 0 && i < this->area())
#endif
		{
			Cell_T& _cell = this->grid[i];
			_cell.val += AccVal<1>();
			if (_cell.val > this->max_weight) this->max_weight = _cell.val;

			const FloatT _z = static_cast<FloatT>(pt.z);
			if (_z < _cell.min_z) _cell.min_z = _z;
			else if (_z > _cell.max_z) _cell.max_z = _z;
			_cell.avg_z = (static_cast<FloatT>(_cell.val - AccVal<1>()) * _cell.avg_z + _z) / static_cast<FloatT>(_cell.val);

			return true;
		}
		return false;
	}

protected:
	Eigen::Vector2<FloatT> map_origin{};
	Eigen::Vector2<IntT> map_size{};
	FloatT resolution{ 1.f };
	Accumulation_T max_weight{ AccVal<0>() };
	Cell_T* grid{ nullptr };

};
