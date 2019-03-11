import cx from "classnames";
import React from "react";

import "./sideMenu.scss";

export default class SideMenu extends React.Component {
  render() {
    return (
      <div className="side-menu">
        <div className="title">LIDARS</div>
        <ul>
          {this.props.items.map(item => {
            const selected = item.id == this.props.selectedId;
            const classNames = cx("item", { "selected": selected });
            return (
              <li className={classNames} key={item.id} data-id={item.id} onClick={this.props.handleClick}>
                {item.id}
              </li>
            );
          })}
        </ul>
      </div>
    )
  }
};
