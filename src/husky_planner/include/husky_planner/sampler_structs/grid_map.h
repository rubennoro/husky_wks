#pragma once
#include <cstdint>
#include "husky_planner/params.h"

/**
 * This file contains the in-house 2.5 GridMap for sampling points on the ground.
 * The GridMap is formatted as a 2D Matrix of cells, with access to 
 * (x, y, z) information as well as info related to sampling, like the 
 * number_nodes that have been sampled within that specific cell, and 
 * the density, which 
 * TODO(): COMPLETE THIS.
 */


static constexpr int Dynamic = -1;

/**
 * Matrix Class is used for storing an array of elements with size defined
 * either at compile-time or run-time. The underlying data structure serves
 * as a 2D array with memory contiguity in 1D for simplicity. 
 */
template<typename T, int rows = Dynamic, int cols = Dynamic>
class Matrix{
public:
    Matrix() = default;
    /**
     * Compile-time constructor for matrix.
     */
    Matrix(float resolution){
        static_assert(rows != Dynamic && cols != Dynamic,
                      "Default constructor only allowed for fixed-size matrices");
        res_ = resolution;
        // TODO(): Add a check here to make sure that the resolution evenly divides the rows/cols.
        rows_ = static_cast<int>(static_cast<float>(rows) / res_);
        cols_ = static_cast<int>(static_cast<float>(cols) / res_);
        data_ = new T[rows_ * cols_]();
    }

    /** 
     * Run-time constructor for matrix.
     */
    Matrix(int num_rows, int num_cols, float resolution){
        res_ = resolution;

        // TODO(): Add a check here to make sure that the resolution evenly divides the rows/cols.
        rows_ = static_cast<int>(static_cast<float>(num_rows) / res_);
        cols_ = static_cast<int>(static_cast<float>(num_cols) / res_);
        data_ = new T[rows_ * cols_]();
    }

    /**
     * Const access override operator for 2D access.
     */
    const T& operator()(int x, int y) const{
        return data_[x * cols_ + y];
    }

    /**
     * Access override operator for 2D access.
     */
    T& operator()(int x, int y){
        return data_[x * cols_ + y];
    }

    uint32_t get_rows() const{
        return rows_;
    }
    uint32_t get_cols() const{
        return cols_;
    }
    float res() const{
        return res_;
    }
    ~Matrix(){
        delete[] data_;
    }
private:
    int rows_;
    int cols_;
    float res_;
    T* data_; 
};

/*
 * An individual cell in the grid for sampling. Contains the 2D position,
 * the number of currently sampled nodes from it, and the density for distribution sampling. 
 */
class Cell{
public:
    Cell(): x_(0), y_(0), z_(0), num_nodes_(0), density_(0){} 
    Cell(float x, float y, float z): x_(x), y_(y), z_(z){}
    ~Cell() = default;

    float x() const{
        return x_;
    }

    float y() const{
        return y_;
    }

    float z() const{
        return z_;
    }

    void set_x(float x){
        x_ = x;
    }

    void set_y(float y){
        y_ = y;
    }

    void set_z(float z){
        z_ = z;
    }

    uint32_t nodes() const{
        return num_nodes_;
    }

    void add_node(){
        num_nodes_++;
    }

    float density() const{
        return density_;
    }

    void update_density(float updated){
        density_ = updated;
    }
private:
    /*
     * The x index within the gridmap the cell occupies.
     */
    float x_;
    /*
     * The y index within the gridmap the cell occupies.
     */
    float y_;
    /*
     * The height at (x,y).
     */
    float z_;

    /*
     * The number of nodes randomly sampled by this cell.
     */
    uint32_t num_nodes_;

    /*
     * The density associated with this cell, used for building a CDF.
     */
    float density_;
};

/*
 * Size of body to operate over grid for updating sampling pdf. 
 */
struct Kernel{
    Kernel(float x, float y): d_x(x), d_y(y){}
    float d_x;
    float d_y;
};


/**
 * GridMap class stores a 2D Matrix of Cells containing information about indices
 * and node count information for the adaptive sampling distribution.
 */
class GridMap{
public:

    GridMap(int row_size, int col_size, float res = Params::sampling.res): map_(row_size, col_size, res){
        total_cells = static_cast<int>(static_cast<float>(row_size) / res * static_cast<float>(col_size) / res);
        
        /*
         * These values are accurately determining during initialization. 
         * The num_elev_cells will increment, while num_ground_cells will decrement during Sampler::init_height();
         */
        num_ground_cells = total_cells;
        num_elev_cells = 0;
    }

    Cell& operator()(int x, int y){
        return map_(x, y);
    }

    const Cell& operator()(int x, int y) const{
        return map_(x, y);
    }

    Cell& index_cell(int idx) {
        int x = idx / cols();
        int y = idx % cols();
        return (*this)(x, y);
    }

    uint32_t rows() const{
        return map_.get_rows();
    }
    uint32_t cols() const{
        return map_.get_cols();
    }

    void inc_elev_cells(){
        if(num_elev_cells >= total_cells){
            return;
        }
        num_elev_cells++;
    }

    void dec_elev_cells(){
        if(num_elev_cells <= 0){
            return;
        }
        num_elev_cells--;
    }

    void inc_ground_cells(){
        if(num_ground_cells >= total_cells){
            return;
        }
        num_ground_cells++;
    }
    void dec_ground_cells(){
        if(num_ground_cells <= 0){
            return;
        }
        num_ground_cells--;
    }

    /**
     * TODO(): 
     * Given the resolution and the rows / cols, provide each index with an associated x,y value to hold 
     * These should specify the cell's characteristics 
     * 
     * Each Cell in the grid is like:
     * 
     */
    void init_coords(){

        for(uint32_t i = 0; i < rows(); i++){
            for(uint32_t j = 0; j < cols(); j++){
                /*
                 * These values are the mins, they go up to the res. 
                 * Height is automatically set within the constructor.
                 */
                map_(i, j).set_x(i * map_.res());
                map_(i, j).set_y(j * map_.res());
                map_(i, j).set_z(0);
            }
        }
    }
private:
    int total_cells;
    int num_ground_cells;
    int num_elev_cells;
    /*
     * A Matrix may be defined at compile-time, but in this case it is a run-time variable.
     */
    Matrix<Cell, Dynamic, Dynamic> map_;
};
