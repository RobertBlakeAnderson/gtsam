/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Conditional.h
 * @brief   Base class for conditional densities
 * @author  Frank Dellaert
 */

// \callgraph
#pragma once

#include <iostream>

#include <gtsam/inference/Conditional.h>

namespace gtsam {

  /* ************************************************************************* */
  template<class FACTOR, class DERIVEDFACTOR>
  void Conditional<FACTOR,DERIVEDFACTOR>::print(const std::string& s,
      const KeyFormatter& formatter,
      std::ostream& os) const {
    os << s << " P(";
    for(Key key: frontals())
      os << " " << formatter(key);
    if (nrParents() > 0)
      os << " |";
    for(Key parent: parents())
      os << " " << formatter(parent);
    os << ")" << std::endl;
  }

  /* ************************************************************************* */
  template<class FACTOR, class DERIVEDFACTOR>
  bool Conditional<FACTOR,DERIVEDFACTOR>::equals(const This& c, double tol) const
  {
    return nrFrontals_ == c.nrFrontals_;
  }

}
