#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace base_classes{

  class PolygonBase{
  public:
    virtual void initialize(double side_length) = 0;
    virtual double area() = 0;
    virtual ~PolygonBase(){}

  protected:
  // A constructor without parameters is required, 
  // so if any parameters to the class are needed, 
  // we use the initialize method to pass them to the object.
    PolygonBase(){}
  };
}  // namespace base_class

#endif  // POLYGON_BASE_REGULAR_POLYGON_HPP