import React from "react";

import SideMenu from "../components/SideMenu";
import PointCloudView from "../components/PointCloudView";

import "./lidarFleet.scss";

export default class LidarFleet extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      itemId: this.props.itemId
    };

    this.onItemClick = this.onItemClick.bind(this);
  }

  render() {
    // TODO: Get available lidars from server
    const items = [
      {id: "foo"},
      {id: "baz"},
      {id: "bar"}
    ];
    return (
      <div className="lidar-fleet">
        <SideMenu items={items} handleClick={this.onItemClick} selectedId={this.state.itemId} />
        <PointCloudView lidarId={this.state.itemId} />
      </div>
    )
  }

  onItemClick(e) {
    this.setState({itemId: e.target.dataset.id});
  }
};
