#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base{

  class RegularPolygon{
  public:
    virtual void initialize(double side_length) = 0;
    virtual double area() = 0;
    virtual ~RegularPolygon(){}

  protected:
  // A constructor without parameters is required, 
  // so if any parameters to the class are needed, 
  // we use the initialize method to pass them to the object.
    RegularPolygon(){}
  };
}  // namespace polygon_base

#endif  // POLYGON_BASE_REGULAR_POLYGON_HPP