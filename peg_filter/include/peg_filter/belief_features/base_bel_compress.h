#ifndef BASE_BEL_COMPRESS_H_
#define BASE_BEL_COMPRESS_H_

class BaseBeliefCompression{

public:

   BaseBeliefCompression(){name="NONE";}

   BaseBeliefCompression(const std::string& name):name(name){}

   virtual void update(const arma::colvec3& velocity, const arma::mat& points, const arma::cube& P) = 0;

   const std::string& get_name() const{
       return name;
   }

protected:

   std::string name;

};

#endif
