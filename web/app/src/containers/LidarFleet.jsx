import React from "react";

import SideMenu from "../components/SideMenu";
import PointCloudView from "../components/PointCloudView";

import "./lidarFleet.scss";

export default class LidarFleet extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      itemId: this.props.itemId,
      lidars: []
    };

    this.onItemClick = this.onItemClick.bind(this);
  }

  render() {
    return (
      <div className="lidar-fleet">
        <SideMenu items={this.state.lidars} handleClick={this.onItemClick} selectedId={this.state.itemId} />
        <PointCloudView lidarId={this.state.itemId} />
      </div>
    )
  }

  componentDidMount() {
    const host = window.document.location.host.replace(/:.*/, '');
    const port = process.env.PORT || 3002;
    this.socket = new WebSocket('ws://' + host + ':' + port);
    this.socket.onmessage = (e) => {
      const msg = JSON.parse(e.data);
      switch(msg.type) {
        case 'lidars':
          this.setState({lidars: msg.lidars});
      }
    };
  }

  onItemClick(e) {
    this.setState({itemId: e.target.dataset.id});
  }
};
