// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "InfoBoxes/Content/Task.hpp"
#include "InfoBoxes/Panel/Panel.hpp"
#include "InfoBoxes/Data.hpp"
#include "Interface.hpp"
#include "Components.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "Dialogs/Waypoint/WaypointDialogs.hpp"
#include "Engine/Util/Gradient.hpp"
#include "Engine/Waypoint/Waypoint.hpp"
#include "Units/Units.hpp"
#include "Formatter/TimeFormatter.hpp"
#include "Language/Language.hpp"
#include "Widget/CallbackWidget.hpp"
#include "Renderer/NextArrowRenderer.hpp"
#include "UIGlobals.hpp"
#include "Look/Look.hpp"
#include "ui/canvas/Canvas.hpp"

#include "LogFile.hpp"
#include "Engine/Task/Ordered/OrderedTask.hpp"
#include "Engine/Task/Ordered/Points/StartPoint.hpp"
#include "Engine/Task/TaskManager.hpp"
#include "Engine/GlideSolvers/MacCready.hpp"
#include "Task/ObservationZones/LineSectorZone.hpp"
#include "GlideSolvers/GlideState.hpp"
#include "NMEA/Aircraft.hpp"
#include "Formatter/UserUnits.hpp"
#include "Geo/Math.hpp"
#include "BackendComponents.hpp"
#include "DataComponents.hpp"

#include <tchar.h>

static void
ShowNextWaypointDetails() noexcept
{
  if (!backend_components->protected_task_manager)
    return;

  auto wp = backend_components->protected_task_manager->GetActiveWaypoint();
  if (wp == nullptr)
    return;

  dlgWaypointDetailsShowModal(data_components->waypoints.get(),
                              std::move(wp), false);
}

static std::unique_ptr<Widget>
LoadNextWaypointDetailsPanel([[maybe_unused]] unsigned id) noexcept
{
  return std::make_unique<CallbackWidget>(ShowNextWaypointDetails);
}

#ifdef __clang__
/* gcc gives "redeclaration differs in 'constexpr'" */
constexpr
#endif
const InfoBoxPanel next_waypoint_infobox_panels[] = {
  { N_("Details"), LoadNextWaypointDetailsPanel },
  { nullptr, nullptr }
};

void
UpdateInfoBoxBearing(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;
  if (!task_stats.task_valid || !vector_remaining.IsValid() ||
      vector_remaining.distance <= 10) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValue(vector_remaining.bearing);
  data.SetValueColor(task_stats.inside_oz ? 3 : 0);
}

void
UpdateInfoBoxBearingDiff(InfoBoxData &data) noexcept
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;
  if (!basic.track_available || !task_stats.task_valid ||
      !vector_remaining.IsValid() || vector_remaining.distance <= 10) {
    data.SetInvalid();
    return;
  }

  Angle Value = vector_remaining.bearing - basic.track;
  data.SetValueFromBearingDifference(Value);
  data.SetValueColor(task_stats.inside_oz ? 3 : 0);
}

void
UpdateInfoBoxRadial(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;
  if (!task_stats.task_valid || !vector_remaining.IsValid() ||
      vector_remaining.distance <= 10) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValue(vector_remaining.bearing.Reciprocal());
  data.SetValueColor(task_stats.inside_oz ? 3 : 0);

  data.SetCommentFromDistance(vector_remaining.distance);
}

void
InfoBoxContentNextWaypoint::Update(InfoBoxData &data) noexcept
{
  // use proper non-terminal next task stats

  const auto way_point = backend_components->protected_task_manager
    ? backend_components->protected_task_manager->GetActiveWaypoint()
    : nullptr;

  if (!way_point) {
    data.SetTitle(_("Next"));
    data.SetInvalid();
    return;
  }

  data.SetTitle(way_point->name.c_str());

  // Set Comment
  if (way_point->radio_frequency.IsDefined()) {
    const unsigned freq = way_point->radio_frequency.GetKiloHertz();
    data.FmtComment(_T("{}.{:03} {}"),
                    freq / 1000, freq % 1000, way_point->comment);
  }
  else
    data.SetComment(way_point->comment.c_str());

  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GlideResult &solution_remaining =
    task_stats.current_leg.solution_remaining;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;
  if (!basic.track_available || !task_stats.task_valid ||
      !vector_remaining.IsValid()) {
    data.SetValueInvalid();
    return;
  }

  // Set Value
  Angle Value = vector_remaining.bearing - basic.track;
  data.SetValueFromBearingDifference(Value);

  // Set Color (blue/black)
  data.SetValueColor(solution_remaining.IsFinalGlide() ? 2 : 0);
}

const InfoBoxPanel *
InfoBoxContentNextWaypoint::GetDialogContent() noexcept
{
  return next_waypoint_infobox_panels;
}

void
UpdateInfoBoxNextDistance(InfoBoxData &data) noexcept
{
  const auto way_point = backend_components->protected_task_manager
    ? backend_components->protected_task_manager->GetActiveWaypoint()
    : nullptr;

  // Set title
  if (!way_point)
    data.SetTitle(_("WP Dist"));
  else
    data.SetTitle(way_point->name.c_str());

  // use proper non-terminal next task stats

  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;
  if (!task_stats.task_valid || !vector_remaining.IsValid()) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValueFromDistance(vector_remaining.distance);
  data.SetValueColor(task_stats.inside_oz ? 3 : 0);

  if (basic.track_available) {
    Angle bd = vector_remaining.bearing - basic.track;
    data.SetCommentFromBearingDifference(bd);
  } else
    data.SetCommentInvalid();
}

void
UpdateInfoBoxNextDistanceNominal(InfoBoxData &data) noexcept
{
  const auto way_point = backend_components->protected_task_manager
    ? backend_components->protected_task_manager->GetActiveWaypoint()
    : nullptr;

  if (!way_point) {
    data.SetInvalid();
    return;
  }

  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;

  if (!task_stats.task_valid || !basic.location_available) {
      data.SetInvalid();
      return;
  }

  const GeoVector vector(basic.location, way_point->location);

  if (!vector.IsValid()) {
      data.SetInvalid();
      return;
  }

  // Set Value
  data.SetValueFromDistance(vector.distance);
  data.SetValueColor(task_stats.inside_oz ? 3 : 0);
  data.SetComment(vector.bearing);
}

void
UpdateInfoBoxNextETE(InfoBoxData &data) noexcept
{
  // use proper non-terminal next task stats

  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid || !task_stats.current_leg.IsAchievable()) {
    data.SetInvalid();
    return;
  }

  assert(task_stats.current_leg.time_remaining_now.count() >= 0);

  data.SetValueFromTimeTwoLines(task_stats.current_leg.time_remaining_now);
}

void
UpdateInfoBoxNextETA(InfoBoxData &data) noexcept
{
  // use proper non-terminal next task stats

  const auto &task_stats = CommonInterface::Calculated().task_stats;
  const BrokenTime &now_local = CommonInterface::Calculated().date_time_local;

  if (!task_stats.task_valid || !task_stats.current_leg.IsAchievable() ||
      !now_local.IsPlausible()) {
    data.SetInvalid();
    return;
  }

  const BrokenTime t = now_local +
    std::chrono::duration_cast<std::chrono::seconds>(task_stats.current_leg.solution_remaining.time_elapsed);

  // Set Value
  data.FmtValue(_T("{:02}:{:02}"), t.hour, t.minute);

  // Set Comment
  data.FmtComment(_T("{:02}"), t.second);
}

static void
SetValueFromAltDiff(InfoBoxData &data, const TaskStats &task_stats,
                    const GlideResult &solution)
{
  if (!task_stats.task_valid || !solution.IsAchievable()) {
    data.SetInvalid();
    return;
  }

  const auto &settings = CommonInterface::GetComputerSettings();
  auto altitude_difference =
    solution.SelectAltitudeDifference(settings.task.glide);
  data.SetValueFromArrival(altitude_difference);
}

void
UpdateInfoBoxNextAltitudeDiff(InfoBoxData &data) noexcept
{
  // pilots want this to be assuming terminal flight to this wp

  const auto &task_stats = CommonInterface::Calculated().task_stats;
  const auto &next_solution = task_stats.current_leg.solution_remaining;

  SetValueFromAltDiff(data, task_stats, next_solution);
}

void
UpdateInfoBoxNextMC0AltitudeDiff(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;

  SetValueFromAltDiff(data, task_stats,
                      task_stats.current_leg.solution_mc0);
}

void
UpdateInfoBoxNextAltitudeRequire(InfoBoxData &data) noexcept
{
  // pilots want this to be assuming terminal flight to this wp

  const auto &task_stats = CommonInterface::Calculated().task_stats;
  const auto &next_solution = task_stats.current_leg.solution_remaining;
  if (!task_stats.task_valid || !next_solution.IsAchievable()) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromAltitude(next_solution.GetRequiredAltitude());
}

void
UpdateInfoBoxNextAltitudeArrival(InfoBoxData &data) noexcept
{
  // pilots want this to be assuming terminal flight to this wp

  const auto &basic = CommonInterface::Basic();
  const auto &task_stats = CommonInterface::Calculated().task_stats;
  const auto next_solution = task_stats.current_leg.solution_remaining;
  if (!basic.NavAltitudeAvailable() ||
      !task_stats.task_valid || !next_solution.IsAchievable()) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromAltitude(next_solution.GetArrivalAltitude(basic.nav_altitude));
}


void
UpdateInfoBoxNextGR(InfoBoxData &data) noexcept
{
  // pilots want this to be assuming terminal flight to this wp, and this
  // is what current_leg gradient does.

  if (!CommonInterface::Calculated().task_stats.task_valid) {
    data.SetInvalid();
    return;
  }

  auto gradient = CommonInterface::Calculated().task_stats.current_leg.gradient;

  if (gradient <= 0) {
    data.SetValue(_T("+++"));
    return;
  }
  if (::GradientValid(gradient)) {
    data.SetValueFromGlideRatio(gradient);
  } else {
    data.SetInvalid();
  }
}

void
UpdateInfoBoxFinalDistance(InfoBoxData &data) noexcept
{
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.task_stats;

  if (!task_stats.task_valid ||
      !task_stats.current_leg.vector_remaining.IsValid() ||
      !task_stats.total.remaining.IsDefined()) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValueFromDistance(task_stats.task_finished
                            ? task_stats.current_leg.vector_remaining.distance
                            : task_stats.total.remaining.GetDistance());
}

void
UpdateInfoBoxFinalETE(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;

  if (!task_stats.task_valid || !task_stats.total.IsAchievable()) {
    data.SetInvalid();
    return;
  }

  assert(task_stats.total.time_remaining_now.count() >= 0);

  data.SetValueFromTimeTwoLines(task_stats.total.time_remaining_now);
}

void
UpdateInfoBoxFinalETA(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const BrokenTime &now_local = CommonInterface::Calculated().date_time_local;

  if (!task_stats.task_valid || !task_stats.total.IsAchievable() ||
      !now_local.IsPlausible()) {
    data.SetInvalid();
    return;
  }

  const BrokenTime t = now_local +
    std::chrono::duration_cast<std::chrono::seconds>(task_stats.total.solution_remaining.time_elapsed);

  // Set Value
  data.FmtValue(_T("{:02}:{:02}"), t.hour, t.minute);

  // Set Comment
  data.FmtComment(_T("{:02}"), t.second);
}

void
UpdateInfoBoxFinalAltitudeDiff(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;

  SetValueFromAltDiff(data, task_stats, task_stats.total.solution_remaining);
}

void
UpdateInfoBoxFinalMC0AltitudeDiff(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;

  SetValueFromAltDiff(data, task_stats,
                      task_stats.total.solution_mc0);
}

void
UpdateInfoBoxFinalAltitudeRequire(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid ||
      !task_stats.total.solution_remaining.IsOk()) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromAltitude(task_stats.total.solution_remaining.GetRequiredAltitude());
}

void
UpdateInfoBoxTaskSpeed(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid || !task_stats.total.travelled.IsDefined()) {
    data.SetInvalid();
    return;
  }

  // Set Value and unit
  data.SetValueFromTaskSpeed(task_stats.total.travelled.GetSpeed());
}

void
UpdateInfoBoxTaskSpeedAchieved(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid ||
      !task_stats.total.remaining_effective.IsDefined()) {
    data.SetInvalid();
    return;
  }

  // Set Value and unit
  data.SetValueFromTaskSpeed(task_stats.total.remaining_effective.GetSpeed());
}

void
UpdateInfoBoxTaskSpeedInstant(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid || task_stats.inst_speed_fast < 0 ||
      task_stats.inst_speed_slow < 0) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValueFromTaskSpeed(task_stats.inst_speed_fast);

  // Add slow filtered task speed as comment item
  data.SetCommentFromTaskSpeed(task_stats.inst_speed_slow, false);
}

void
UpdateInfoBoxTaskSpeedHour(InfoBoxData &data) noexcept
{
  const WindowStats &window =
    CommonInterface::Calculated().task_stats.last_hour;
  if (!window.IsDefined()) {
    data.SetInvalid();
    return;
  }

  // Set Value and unit
  data.SetValueFromTaskSpeed(window.speed);
}

void
UpdateInfoBoxTaskSpeedEst(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid || !task_stats.total.planned.IsDefined() ||
      !task_stats.total.IsAchievable()) {
    data.SetInvalid();
    return;
  }

  // Set Value and unit
  data.SetValueFromTaskSpeed(task_stats.total.planned.GetSpeed());
}

void
UpdateInfoBoxFinalGR(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid) {
    data.SetInvalid();
    return;
  }

  auto gradient = task_stats.total.gradient;

  if (gradient <= 0) {
    data.SetValue(_T("+++"));
    return;
  }
  if (::GradientValid(gradient))
    data.SetValueFromGlideRatio(gradient);
  else
    data.SetInvalid();
}

void
UpdateInfoBoxTaskAATime(InfoBoxData &data) noexcept
{
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.ordered_task_stats;
  const CommonStats &common_stats = calculated.common_stats;

  if (!task_stats.has_targets ||
      !task_stats.total.IsAchievable()) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromTimeTwoLines(common_stats.aat_time_remaining);
  data.SetValueColor(common_stats.aat_time_remaining.count() < 0 ? 1 : 0);
}

void
UpdateInfoBoxTaskAATimeDelta(InfoBoxData &data) noexcept
{
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.ordered_task_stats;
  const CommonStats &common_stats = calculated.common_stats;

  if (!task_stats.has_targets ||
      !task_stats.total.IsAchievable()) {
    data.SetInvalid();
    return;
  }

  assert(task_stats.total.time_remaining_start.count() >= 0);

  auto diff = task_stats.total.time_remaining_start -
    common_stats.aat_time_remaining;

  data.SetValueFromTimeTwoLines(diff);
  // Set Color (red/blue/black)
  data.SetValueColor(diff.count() < 0 ? 1 :
                     task_stats.total.time_remaining_start >
                     common_stats.aat_time_remaining + std::chrono::minutes{5} ? 2 : 0);
}

void
UpdateInfoBoxTaskAADistance(InfoBoxData &data) noexcept
{
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.ordered_task_stats;

  if (!task_stats.has_targets ||
      !task_stats.total.planned.IsDefined()) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValueFromDistance(task_stats.total.planned.GetDistance());
}

void
UpdateInfoBoxTaskAADistanceMax(InfoBoxData &data) noexcept
{
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.ordered_task_stats;

  if (!task_stats.has_targets) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValueFromDistance(task_stats.distance_max);
}

void
UpdateInfoBoxTaskAADistanceMin(InfoBoxData &data) noexcept
{
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.ordered_task_stats;

  if (!task_stats.has_targets) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValueFromDistance(task_stats.distance_min);
}

void
UpdateInfoBoxTaskAASpeed(InfoBoxData &data) noexcept
{
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.ordered_task_stats;
  const CommonStats &common_stats = calculated.common_stats;

  if (!task_stats.has_targets || common_stats.aat_speed_target <= 0) {
    data.SetInvalid();
    return;
  }

  // Set Value and units
  data.SetValueFromTaskSpeed(common_stats.aat_speed_target);
}

void
UpdateInfoBoxTaskAASpeedMax(InfoBoxData &data) noexcept
{
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.ordered_task_stats;
  const CommonStats &common_stats = calculated.common_stats;

  if (!task_stats.has_targets || common_stats.aat_speed_max <= 0) {
    data.SetInvalid();
    return;
  }

  // Set Value and units
  data.SetValueFromTaskSpeed(common_stats.aat_speed_max);
}

void
UpdateInfoBoxTaskAASpeedMin(InfoBoxData &data) noexcept
{
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.ordered_task_stats;
  const CommonStats &common_stats = calculated.common_stats;

  if (!task_stats.has_targets ||
      !task_stats.task_valid || common_stats.aat_speed_min <= 0) {
    data.SetInvalid();
    return;
  }

  // Set Value and units
  data.SetValueFromTaskSpeed(common_stats.aat_speed_min);
}

void
UpdateInfoBoxTaskTimeUnderMaxHeight(InfoBoxData &data) noexcept
{
  const auto &calculated = CommonInterface::Calculated();
  const auto &task_stats = calculated.ordered_task_stats;
  const auto &common_stats = calculated.common_stats;
  const double maxheight = backend_components->protected_task_manager->GetOrderedTaskSettings().start_constraints.max_height;

  if (!task_stats.task_valid || maxheight <= 0
      || !backend_components->protected_task_manager
      || !common_stats.TimeUnderStartMaxHeight.IsDefined()) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromTimeTwoLines(CommonInterface::Basic().time -
                                common_stats.TimeUnderStartMaxHeight);
  data.SetComment(_("Time Below"));
}

void
UpdateInfoBoxNextETEVMG(InfoBoxData &data) noexcept
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;

  if (!basic.ground_speed_available || !task_stats.task_valid ||
      !task_stats.current_leg.remaining.IsDefined()) {
    data.SetInvalid();
    return;
  }

  const auto d = task_stats.current_leg.remaining.GetDistance();
  const auto v = basic.ground_speed;

  if (!task_stats.task_valid ||
      v <= 0) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromTimeTwoLines(FloatDuration{d / v});
}

void
UpdateInfoBoxNextETAVMG(InfoBoxData &data) noexcept
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;

  if (!basic.ground_speed_available || !task_stats.task_valid ||
      !task_stats.current_leg.remaining.IsDefined()) {
    data.SetInvalid();
    return;
  }

  const auto d = task_stats.current_leg.remaining.GetDistance();
  const auto v = basic.ground_speed;

  if (!task_stats.task_valid ||
      v <= 0) {
    data.SetInvalid();
    return;
  }

  const BrokenTime &now_local = CommonInterface::Calculated().date_time_local;
  if (now_local.IsPlausible()) {
    const std::chrono::seconds dd{long(d/v)};
    const BrokenTime t = now_local + dd;
    data.FmtValue(_T("{:02}:{:02}"), t.hour, t.minute);
    data.FmtComment(_T("{:02}"), t.second);
  }

}

void
UpdateInfoBoxFinalETEVMG(InfoBoxData &data) noexcept
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;

  if (!basic.ground_speed_available || !task_stats.task_valid ||
      !task_stats.total.remaining.IsDefined()) {
    data.SetInvalid();
    return;
  }

  const auto d = task_stats.total.remaining.GetDistance();
  const auto v = basic.ground_speed;

  if (!task_stats.task_valid ||
      v <= 0) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromTimeTwoLines(FloatDuration{d / v});
}

void
UpdateInfoBoxCruiseEfficiency(InfoBoxData &data) noexcept
{
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid || !task_stats.start.HasStarted()) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromPercent(task_stats.cruise_efficiency*100);
  data.SetCommentFromVerticalSpeed(task_stats.effective_mc, false);
}

static constexpr unsigned
SecondsUntil(TimeStamp now, RoughTime until) noexcept
{
  auto d = TimeStamp{until} - now;
  if (d.count() < 0)
    d += std::chrono::hours{24};
  return std::chrono::duration_cast<std::chrono::duration<unsigned>>(d).count();
}

void
UpdateInfoBoxStartOpen(InfoBoxData &data) noexcept
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.ordered_task_stats;
  const CommonStats &common_stats = CommonInterface::Calculated().common_stats;
  const RoughTimeSpan &open = common_stats.start_open_time_span;

  /* reset color that may have been set by a previous call */
  data.SetValueColor(0);

  if (!basic.time_available || !task_stats.task_valid ||
      common_stats.ordered_summary.active != 0 ||
      !open.IsDefined()) {
    data.SetInvalid();
    return;
  }

  const auto now_s = basic.time;
  const RoughTime now{now_s};

  if (open.HasEnded(now)) {
    data.SetValueInvalid();
    data.SetComment(_("Closed"));
  } else if (open.HasBegun(now)) {
    if (open.GetEnd().IsValid()) {
      unsigned seconds = SecondsUntil(now_s, open.GetEnd());
      data.FmtValue(_T("{:02}:{:02}"), seconds / 60, seconds % 60);
      data.SetValueColor(3);
    } else
      data.SetValueInvalid();

    data.SetComment(_("Open"));
  } else {
    unsigned seconds = SecondsUntil(now_s, open.GetStart());
    data.FmtValue(_T("{:02}:{:02}"), seconds / 60, seconds % 60);
    data.SetValueColor(2);
    data.SetComment(_("Waiting"));
  }
}

void
UpdateInfoBoxStartOpenArrival(InfoBoxData &data) noexcept
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.ordered_task_stats;
  const GlideResult &current_remaining =
    task_stats.current_leg.solution_remaining;
  const CommonStats &common_stats = CommonInterface::Calculated().common_stats;
  const RoughTimeSpan &open = common_stats.start_open_time_span;

  /* reset color that may have been set by a previous call */
  data.SetValueColor(0);

  if (!basic.time_available || !task_stats.task_valid ||
      common_stats.ordered_summary.active != 0 ||
      !open.IsDefined() ||
      !current_remaining.IsOk()) {
    data.SetInvalid();
    return;
  }

  const auto arrival_s = basic.time + current_remaining.time_elapsed;
  const RoughTime arrival{arrival_s};

  if (open.HasEnded(arrival)) {
    data.SetValueInvalid();
    data.SetComment(_("Closed"));
  } else if (open.HasBegun(arrival)) {
    if (open.GetEnd().IsValid()) {
      unsigned seconds = SecondsUntil(arrival_s, open.GetEnd());
      data.FmtValue(_T("{:02}:{:02}"), seconds / 60, seconds % 60);
      data.SetValueColor(3);
    } else
      data.SetValueInvalid();

    data.SetComment(_("Open"));
  } else {
    unsigned seconds = SecondsUntil(arrival_s, open.GetStart());
    data.FmtValue(_T("{:02}:{:02}"), seconds / 60, seconds % 60);
    data.SetValueColor(2);
    data.SetComment(_("Waiting"));
  }
}

/*
 * The StartlineDistance infobox contains information about distance and arrival altitudes at the startline
 * This function updates the text fields in the infobox.
 */
void
InfoBoxStartlineDistance::Update(InfoBoxData &data) noexcept
{
static int count = 0;
  data.SetCustom(count++);
}

/*
 * The StartlineDistance infobox contains information about distance and arrival altitudes at the startline
 * This function renders the arrow.
 */
void
InfoBoxStartlineDistance::OnCustomPaint(Canvas &canvas, const PixelRect &rc) noexcept
{
#define ALT_RED 0
#define ALT_YELLOW -50
#define ALT_GREEN -100

  const Look &look = UIGlobals::GetLook();
  const double max_speed = backend_components->protected_task_manager != nullptr ? backend_components->protected_task_manager->GetOrderedTaskSettings().start_constraints.max_speed : 0;
  const double startline_alt = backend_components->protected_task_manager != nullptr ? backend_components->protected_task_manager->GetOrderedTaskSettings().start_constraints.max_height : 0;


  // get the data to be displayed
  StaticString<32> text1, text2, text3;
  text1 = _T("---");
  text2 = _T("---");
  text3 = _T("---");

  Color color1, color2, color3;
  color1 = (look.info_box.inverse ? COLOR_BLACK : COLOR_WHITE);;
  color2 = (look.info_box.inverse ? COLOR_BLACK : COLOR_WHITE);;
  color3 = (look.info_box.inverse ? COLOR_BLACK : COLOR_WHITE);;

#ifdef mid_value_font
  canvas.Select(look.info_box.mid_value_font);
#endif


  const auto way_point =
      backend_components->protected_task_manager != nullptr ? backend_components->protected_task_manager->GetActiveWaypoint() :
                                          nullptr;


  if (backend_components->protected_task_manager != nullptr) {
    ProtectedTaskManager::Lease task_manager(*backend_components->protected_task_manager);
    const OrderedTask &task = task_manager->GetOrderedTask();

    if (task.TaskSize() > 0) {
      const OrderedTaskPoint &start = task.GetTaskPoint(0);
      const StartPoint *taskpoint_start;

      taskpoint_start =
          start.GetType() == TaskPointType::START ? (const StartPoint *)&start :
                                                    nullptr;

      if (taskpoint_start != nullptr) {
        if ((taskpoint_start->GetObservationZone().GetShape() == ObservationZone::Shape::LINE)
            || (taskpoint_start->GetObservationZone().GetShape() == ObservationZone::Shape::BGA_START)) {
          const MoreData &basic = CommonInterface::Basic();
          const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
          const GlideResult next_solution = task_stats.current_leg.solution_remaining;
          if (!basic.NavAltitudeAvailable() || !task_stats.task_valid
              || !next_solution.IsAchievable()) {
          }
          else {

            // calculate arrival height at start line
            double speed = max_speed + next_solution.head_wind;

            const AircraftState state = ToAircraftState(CommonInterface::Basic(), CommonInterface::Calculated());

//            GeoVector to_startline = start.GetNextLegVector();
//            // the actual distance to the start line
//            to_startline.distance = ProjectedDistance(state.location, to_startline.EndPoint(state.location), start.GetLocation());
//
//            LogDebug(_T("%f"), to_startline.distance);

            const GlideState gs = GlideState::Remaining(start, state, startline_alt);
            GlideSettings settings;
            settings.SetDefaults();
            MacCready mc(settings, CommonInterface::GetComputerSettings().polar.glide_polar_task);

            // all data collected to calculate arrival altitudes at start line
            GlideResult solve = mc.SolveGlide(gs, speed, false);
            double alt_vmax = solve.altitude_difference;
            double alt_MCact = mc.SolveGlide(gs, CommonInterface::Calculated().common_stats.V_block, false).altitude_difference;

            // now fill the text for the fields
            StaticString<32> buffer, buffer2;
            FormatUserDistance(task_stats.current_leg.vector_remaining.distance, buffer.buffer(), true, 2);
            text1.Format(_T("%s"), buffer.c_str());

            FormatUserAltitude(startline_alt+alt_MCact, buffer.buffer(), true);
            FormatUserSpeed(CommonInterface::Calculated().common_stats.V_block, buffer2.buffer(), !true);
            text2.Format(_T("%s @ %s"), buffer.c_str(), buffer2.c_str());

            FormatUserAltitude(startline_alt+alt_vmax, buffer.buffer(), true);
            FormatUserSpeed(speed, buffer2.buffer(), !true);
            text3.Format(_T("%s @ %s"), buffer.c_str(), buffer2.c_str());

            color1 = (look.info_box.inverse ? COLOR_BLACK : COLOR_WHITE);

            if (alt_MCact > ALT_RED)
              color2 = COLOR_RED;
            else if (alt_MCact > ALT_YELLOW)
              color2 = COLOR_YELLOW;
            else if (alt_MCact > ALT_GREEN)
              color2 = COLOR_GREEN;
            else
              color2 = (look.info_box.inverse ? COLOR_BLACK : COLOR_WHITE);

            if (alt_vmax > ALT_RED)
              color3 = COLOR_RED;
            else if (alt_vmax > ALT_YELLOW)
              color3 = COLOR_YELLOW;
            else if (alt_vmax > ALT_GREEN)
              color3 = COLOR_GREEN;
            else
              color3 = (look.info_box.inverse ? COLOR_BLACK : COLOR_WHITE);
          }
        }
      }

    }
  }
  else {
  }


  const auto center = rc.GetCenter();

  int w = rc.GetWidth();
  int h = rc.GetHeight()+5;

  int strip_h = h/3;
  int l = center.x-w/2;
  int r = center.x+w/2+1;

  int y1 = center.y-strip_h*3/2;
  int y2 = center.y-strip_h*1/2;
  int y3 = center.y+strip_h*1/2;
  int y4 = center.y+strip_h*3/2+1;

  int text_y1 = (y1+y2)/2-(canvas.GetFontHeight()/2);
  int text_y2 = (y2+y3)/2-(canvas.GetFontHeight()/2);
  int text_y3 = (y3+y4)/2-(canvas.GetFontHeight()/2);


  // upper line for distance to startline
  canvas.DrawFilledRectangle({l,  y1, r, y2}, color1);
  canvas.SetTextColor((color1==COLOR_BLACK) ? COLOR_WHITE : COLOR_BLACK);
  canvas.DrawText({center.x-(int)canvas.CalcTextWidth(text1)/2, text_y1}, text1);

  // middle line for arrival altitude at actual MC
  canvas.DrawFilledRectangle({l,  y2, r, y3}, color2);
  canvas.SetTextColor((color2==COLOR_BLACK) ? COLOR_WHITE : COLOR_BLACK);
  canvas.DrawText({center.x-(int)canvas.CalcTextWidth(text2)/2, text_y2}, text2);

  // bottom line for arrival altitude at maximum allowed speed
  canvas.DrawFilledRectangle({l,  y3, r, y4}, color3);
  canvas.SetTextColor((color3==COLOR_BLACK) ? COLOR_WHITE : COLOR_BLACK);
  canvas.DrawText({(int)center.x-(int)canvas.CalcTextWidth(text3)/2, text_y3}, text3);

}




/*
 * The NextArrow infobox contains an arrow pointing at the next waypoint.
 * This function updates the text fields in the infobox.
 */
void
InfoBoxContentNextArrow::Update(InfoBoxData &data) noexcept
{
  // use proper non-terminal next task stats
  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;

  // Check if data is valid
  bool distance_valid = task_stats.task_valid && vector_remaining.IsValid();
  bool angle_valid = distance_valid && basic.track_available;

  // Set title. Use waypoint name if available.
  const auto way_point = backend_components->protected_task_manager
    ? backend_components->protected_task_manager->GetActiveWaypoint()
    : nullptr;
  if (!way_point)
    data.SetTitle(_("Next arrow"));
  else
    data.SetTitle(way_point->name.c_str());

  // Set value
  if (angle_valid)
    // Enables OnCustomPaint
    // TODO: use an appropriate digest
    data.SetCustom(basic.track_available.ToInteger());
  else
    data.SetInvalid();

  // Set comment
  if (distance_valid)
    data.SetCommentFromDistance(vector_remaining.distance);
  else
    data.SetCommentInvalid();
}

/*
 * The NextArrow infobox contains an arrow pointing at the next waypoint.
 * This function renders the arrow.
 */
void
InfoBoxContentNextArrow::OnCustomPaint(Canvas &canvas,
                                       const PixelRect &rc) noexcept
{
  // use proper non-terminal next task stats
  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;

  // We exit immediately if this function is called when all data isn't
  // available. This can happen e.g. while state is being changed.
  if (!task_stats.task_valid
      || !vector_remaining.IsValid()
      || !basic.track_available)
    return;

  Angle bd = vector_remaining.bearing - basic.track;

  NextArrowRenderer renderer(UIGlobals::GetLook().wind_arrow_info_box);
  renderer.DrawArrow(canvas, rc, bd);
}

/*
 * This infobox shows either AAT dT + ETA, or only ETA depending on task type
 */
void
UpdateInfoTaskETAorAATdT(InfoBoxData& data) noexcept
{
  const auto& calculated = CommonInterface::Calculated();
  const TaskStats& task_stats = calculated.ordered_task_stats;

  // Always call the ETA infobox function. If task is AAT, the value of
  // ETA infobox will be used as the comment of AATdT infobox
  UpdateInfoBoxFinalETA(data);
  if (task_stats.has_targets) { // Is AAT
    // save the HH:MM ETA to use it as a comment of AATdT infobox
    auto eta_text = data.value;
    UpdateInfoBoxTaskAATimeDelta(data);
    data.SetComment(eta_text);

    data.SetTitle(_T("AAT delta time"));
  } else
    data.SetTitle(_T("Task arrival time"));
}
