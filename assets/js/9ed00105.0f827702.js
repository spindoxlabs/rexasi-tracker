"use strict";(self.webpackChunkrexasi_tracker=self.webpackChunkrexasi_tracker||[]).push([[873],{8633:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>c,contentTitle:()=>o,default:()=>h,frontMatter:()=>a,metadata:()=>r,toc:()=>u});const r=JSON.parse('{"id":"configuration","title":"Configuration","description":"The configuration is a yaml file containing all the parameters necessary to run the tracker. \\\\","source":"@site/docs/configuration.md","sourceDirName":".","slug":"/configuration","permalink":"/docs/configuration","draft":false,"unlisted":false,"editUrl":"https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/docs/configuration.md","tags":[],"version":"current","sidebarPosition":3,"frontMatter":{"sidebar_position":3},"sidebar":"documentationSidebar","previous":{"title":"Getting started","permalink":"/docs/getting-started"},"next":{"title":"Example","permalink":"/docs/example"}}');var s=n(4848),i=n(8453);const a={sidebar_position:3},o="Configuration",c={},u=[];function d(e){const t={br:"br",code:"code",h1:"h1",header:"header",p:"p",pre:"pre",...(0,i.R)(),...e.components};return(0,s.jsxs)(s.Fragment,{children:[(0,s.jsx)(t.header,{children:(0,s.jsx)(t.h1,{id:"configuration",children:"Configuration"})}),"\n",(0,s.jsxs)(t.p,{children:["The configuration is a ",(0,s.jsx)(t.code,{children:"yaml"})," file containing all the parameters necessary to run the tracker. ",(0,s.jsx)(t.br,{}),"\n","The tracker loads the configuration from ",(0,s.jsx)(t.code,{children:"ros/rexasi_tracker/config/config.yaml"}),". ",(0,s.jsx)(t.br,{}),"\n","If no configuration file is found, or it contains a subset of parameters, the default values are used."]}),"\n",(0,s.jsx)(t.p,{children:"Example of configuration:"}),"\n",(0,s.jsx)(t.pre,{children:(0,s.jsx)(t.code,{className:"language-yaml",children:'# General parameters\ngeneral:\n  # Enable/disable debugging functions, it produces more logs\n  debug: true\n\n# This section defines the input/output topics of the rexasi-tracker\ntopics:\n  # The topic on which the rexasi-tracker subscribes waiting for detections\n  tracker_input_topic: "/detections"\n  # The topic on which the rexasi-tracker publishes the output tracks\n  fusion_output_topic: "/tracks"\n\n# This section includes the parameter used by the track fusion algorithm\nfusion_parameters:\n  # Tracks with distance less than tracks_distance_threshold are fused together\n  tracks_distance_threshold: 1.0\n  # Parameter used by the Hungarian algorithm to compute the association matrix\n  hungarian_threshold: 0.5\n\n# This section defines the parameters related to each sensor.\n# If the sensor_id of a detection is present in this list, these parameters override the default ones\nsensors:\n  # The ID of the sensor (defined and published by each detector), must be unique\n  - sensor_id: 1\n    # Tracker parameters (ref. https://tryolabs.github.io/norfair/2.0/reference/tracker/)\n    tracker_paramterers:\n      distance_function: \'euclidean\'\n      distance_threshold: 0.8\n      initialization_delay: 4\n      hit_counter_max: 7\n      pointwise_hit_counter_max: 7\n    # Kalman filter parameters used in the track fusion algorithm (ref. https://filterpy.readthedocs.org)\n    specs:\n      R_std:\n        x: 0.001\n        y: 0.001\n      Q_std:\n        x: 0.04\n        y: 0.04\n'})})]})}function h(e={}){const{wrapper:t}={...(0,i.R)(),...e.components};return t?(0,s.jsx)(t,{...e,children:(0,s.jsx)(d,{...e})}):d(e)}},8453:(e,t,n)=>{n.d(t,{R:()=>a,x:()=>o});var r=n(6540);const s={},i=r.createContext(s);function a(e){const t=r.useContext(i);return r.useMemo((function(){return"function"==typeof e?e(t):{...t,...e}}),[t,e])}function o(e){let t;return t=e.disableParentContext?"function"==typeof e.components?e.components(s):e.components||s:a(e.components),r.createElement(i.Provider,{value:t},e.children)}}}]);