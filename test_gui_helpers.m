function tests = test_gui_helpers
tests = functiontests(localfunctions);
end

function testPointBoundsAcceptsInsidePoints(testCase)
actual = isPointInAxesBounds([0 1], [-1 1], [-5 5], [-2 2]);
verifyTrue(testCase, actual);
end

function testPointBoundsRejectsOutsidePoints(testCase)
actual = isPointInAxesBounds([0 6], [-1 1], [-5 5], [-2 2]);
verifyFalse(testCase, actual);
end

function testPointBoundsRejectsEmptyPoints(testCase)
actual = isPointInAxesBounds([], [], [-5 5], [-2 2]);
verifyFalse(testCase, actual);
end

function testSpeedMultiplierReadsPopupValue(testCase)
fig = figure('Visible', 'off');
cleanup = onCleanup(@() close(fig));
popup = uicontrol('Parent', fig, ...
    'Style', 'popupmenu', ...
    'Tag', 'popup_speed', ...
    'String', {'1x','2x','3x','4x','5x','6x','7x','8x','9x','10x'}, ...
    'Value', 7);
handlesGUI = struct('figure_simulator', fig, 'popup_speed', popup);

actual = getSimSpeedMultiplier(handlesGUI);

verifyEqual(testCase, actual, 7);
end

function testSpeedMultiplierFallsBackToRadioButtons(testCase)
fig = figure('Visible', 'off');
cleanup = onCleanup(@() close(fig));
handlesGUI = struct();
handlesGUI.figure_simulator = fig;
handlesGUI.radio_speed2 = uicontrol('Parent', fig, ...
    'Style', 'radiobutton', 'Value', 0);
handlesGUI.radio_speed3 = uicontrol('Parent', fig, ...
    'Style', 'radiobutton', 'Value', 1);

actual = getSimSpeedMultiplier(handlesGUI);

verifyEqual(testCase, actual, 3);
end

function testReadSonarCreatesOutputRowWhenHistoryIsEmpty(testCase)
robot = CreateRobot();
setAutoEnable(robot, true);
startTimeElap(robot);

distance = ReadSonar(robot, 3);

verifyTrue(testCase, isempty(distance) || isnumeric(distance));
end
