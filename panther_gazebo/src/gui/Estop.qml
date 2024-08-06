// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.3
import "qrc:/qml"

Rectangle
{
  id: widgetRectangle
  color: "white"
  anchors.fill: parent
  focus: true
  Layout.minimumWidth: 400
  Layout.minimumHeight: 225

  Rectangle
  {
    id: eStopRectangle
    border.width: 2
    anchors.top: widgetRectangle.top
    anchors.left: widgetRectangle.left
    focus: true
    height: 175
    width: 400

    // Robot namespace input
    Label {
      id: namespaceLabel
      text: "Namespace:"
      Layout.fillWidth: true
      Layout.margins: 10
      anchors.top: eStopRectangle.top
      anchors.topMargin: 10
      anchors.left: eStopRectangle.left
      anchors.leftMargin: 10
    }

    TextField {
      id: nameField
      width: 175
      Layout.fillWidth: true
      Layout.margins: 10
      text: Estop.namespace
      placeholderText: qsTr("Robot namespace")
      anchors.top: namespaceLabel.bottom
      anchors.topMargin: 5
      anchors.left: eStopRectangle.left
      anchors.leftMargin: 10
      onEditingFinished: {
        Estop.SetNamespace(text)
      }
    }

    ToolButton {
      id: eStopButton
      anchors.bottom: eStopRectangle.bottom
      anchors.bottomMargin: 15
      anchors.horizontalCenter: eStopRectangle.horizontalCenter
      checkable: true
      checked: false
      contentItem: Rectangle {
          width: 72
          height: 72
          radius: 10
          color: eStopButton.checked ? "red" : "green"

          Text {
              anchors.centerIn: parent
              text: eStopButton.checked ? "STOP" : "GO"
              font.bold: true
              color: "white"
          }
      }
      onPressed: {
          Estop.buttonPressed(eStopButton.checked);
      }
    }
  }
}
