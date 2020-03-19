#pragma once

#include "lib7842/api/gui/odom.hpp"

namespace lib7842::GUI {

class Path : public Odom {
public:
  using Page::Page;

  /**
   * Initialize the odom page.
   */
  void initialize() override;

  /**
   * Render the odom page.
   */
  void render() override;

  