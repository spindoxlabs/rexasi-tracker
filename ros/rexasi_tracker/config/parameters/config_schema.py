
{
    'general': {
        'type': 'dict',
        'default': {},
        'schema': {
          'debug': {
            'default': False,
            'type': 'boolean'
          }
        }
    },
    'topics': {
      'type': 'dict',
      'default': {},
      'schema': {
        'tracker_input_topic': {
            'type': 'string',
            'default': 'input/detections'
          },
        'tracker_output_topic': {
          'type': 'string',
          'default': 'tracker/output'
        },
        'fusion_output_topic': {
          'type': 'string',
          'default': 'output/tracks'
        }
      }
    },
    'fusion_parameters': {
        'required': True,
        'type': 'dict',
        'schema': {
          'tracks_distance_threshold': {
            'type': 'number',
            'default': 1.0
          },
          'hungarian_threshold': {
            'type': 'number',
            'default': 0.5
          }
        }
    },
    'sensors': {
        'required': True,
        'type': 'list',
          'schema': {
            'type': 'dict',
            'schema': {
              'sensor_id': {
                'type': 'number',
                'required': True
              },
              'tracker_parameters': {
                'type': 'dict',
                'default': {},
                'schema': {
                  'distance_function': {
                    'type': 'string',
                    'default': 'euclidean'
                  },
                  'distance_threshold': {
                    'type': 'number',
                    'default': 0.8
                  },
                  'initialization_delay': {
                    'type': 'number',
                    'default': 4
                  },
                  'hit_counter_max': {
                    'type': 'number',
                    'default': 7
                  },
                  'pointwise_hit_counter_max': {
                    'type': 'number',
                    'default': 7
                  }
                }
            },
            'specs': {
              'type': 'dict',
              'default': {},
              'schema': {
                'R_std': {
                  'type': 'dict',
                  'default': {},
                  'schema': {
                    'x': {
                      'type': 'number',
                      'default': 0.001
                    },
                    'y': {
                      'type': 'number',
                      'default': 0.001
                    }
                  }
                },
                'Q_std': {
                  'type': 'dict',
                  'default': {},
                  'schema': {
                    'x': {
                      'type': 'number',
                      'default': 0.04
                    },
                    'y': {
                      'type': 'number',
                      'default': 0.04
                    }
                  }
                }
              }
            }
          }
        }
    }
}