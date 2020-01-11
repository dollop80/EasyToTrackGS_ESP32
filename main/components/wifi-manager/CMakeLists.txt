if(IDF_VERSION_MAJOR GREATER_EQUAL 4)
    idf_component_register(SRCS "src/dns_server.c"
						"src/http_server.c"
						"src/json.c"
						"src/wifi_manager.c"
		SRC_DIRS src
        REQUIRES log nvs_flash mdns wpa_supplicant lwip esp_http_server
        INCLUDE_DIRS src
        EMBED_FILES src/style.css src/jquery.gz src/code.js src/index.html)
else()
    set(COMPONENT_SRCDIRS src)
    set(COMPONENT_ADD_INCLUDEDIRS src)
    set(COMPONENT_REQUIRES log nvs_flash mdns wpa_supplicant lwip esp_http_server)
	set(COMPONENT_SRCS "src/dns_server.c"
						"src/http_server.c"
						"src/json.c"
						"src/wifi_manager.c"
	)
    register_component()
endif()
