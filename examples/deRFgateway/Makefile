CONTIKI_PROJECT = deRFgateway

all: $(CONTIKI_PROJECT)

UIP_CONF_IPV6=1
UIP_CONF_RPL=0

CONTIKI = ../..

${info INFO: compiling with CoAP-07}
CFLAGS += -DWITH_COAP=7
CFLAGS += -DREST=coap_rest_implementation
CFLAGS += -DUIP_CONF_TCP=0

APPS = dhcpc-client
APPS += er-coap-07
APPS += REST-server
APPS += erbium

include $(CONTIKI)/Makefile.include
