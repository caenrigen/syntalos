if 'camera-arv' in modules_enabled
    camera_arv_decode_benchmark = executable(
        'benchmark-camera-arv-decode',
        [
            'benchmark-camera-arv-decode.cpp',
            '../../modules/camera-arv/qarv/decoders/swscaledecoder.cpp',
        ],
        include_directories: include_directories(
            '../..',
            '../../modules/camera-arv',
        ),
        dependencies: [
            syntalos_datactl_dep,
            symemopt_dep,
            thread_dep,
            qt_core_dep,
            qt_gui_dep,
            aravis_dep,
            avcodec_dep,
            avutil_dep,
            swscale_dep,
            opencv_dep,
        ],
    )

    benchmark(
        'sy-benchmark-camera-arv-decode',
        camera_arv_decode_benchmark,
        args: [
            '--format', 'all',
            '--width', '1920',
            '--height', '1080',
            '--iterations', '500',
            '--repetitions', '5',
            '--retained', '32',
            '--free-thread', 'both',
        ],
        timeout: 300,
    )
endif
