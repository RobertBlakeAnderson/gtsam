/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Factor.cpp
 * @brief   The base class for all factors
 * @author  Kai Ni
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

// \callgraph

#include <iostream>

#include <gtsam/inference/Factor.h>

namespace gtsam {

  /* ************************************************************************* */
  void Factor::print(const std::string& s, const KeyFormatter& formatter, std::ostream& os) const
  {
    return this->printKeys(s, formatter, os);
  }

  /* ************************************************************************* */
  void Factor::printKeys(const std::string& s, const KeyFormatter& formatter, std::ostream& os) const {
    os << (s.empty() ? "" : s + " ");
    for (Key key : keys_) os << " " << formatter(key);
    os << std::endl;
  }

  /* ************************************************************************* */
  bool Factor::equals(const This& other, double tol) const {
    return keys_ == other.keys_;
  }

}
