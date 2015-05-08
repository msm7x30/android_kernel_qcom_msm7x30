obj-y := msm_fb.o

obj-$(CONFIG_FB_MSM_LOGO) += logo.o

ifeq ($(CONFIG_FB_MSM_MDP_HW),y)
# MDP
obj-y += mdp.o

obj-$(CONFIG_DEBUG_FS) += mdp_debugfs.o

ifeq ($(CONFIG_FB_MSM_MDP40),y)
obj-y += mdp4_util.o
endif

ifeq ($(CONFIG_FB_MSM_OVERLAY),y)
obj-y += mdp4_overlay.o
obj-$(CONFIG_FB_MSM_LCDC) += mdp4_overlay_lcdc.o
obj-y += mdp4_overlay_mddi.o
else
obj-y += mdp_dma_lcdc.o
endif

ifeq ($(CONFIG_FB_MSM_DTV),y)
obj-y += mdp4_dtv.o
obj-y += mdp4_overlay_dtv.o
endif

obj-y += mdp_dma.o
obj-y += mdp_dma_s.o
obj-y += mdp_vsync.o
obj-y += mdp_cursor.o
obj-y += mdp_dma_tv.o

# LCDC
obj-$(CONFIG_FB_MSM_LCDC) += lcdc.o

# MDDI
msm_mddi-objs := mddi.o mddihost.o mddihosti.o
obj-$(CONFIG_FB_MSM_MDDI) += msm_mddi.o

# TVEnc
obj-$(CONFIG_FB_MSM_TVOUT) += tvenc.o
ifeq ($(CONFIG_FB_MSM_OVERLAY),y)
obj-$(CONFIG_FB_MSM_TVOUT) += mdp4_overlay_atv.o
endif

# MSM FB Panel
obj-y += msm_fb_panel.o

ifeq ($(CONFIG_FB_MSM_MDDI_AUTO_DETECT),y)
obj-y += mddi_novatek_fwvga.o
obj-y += mddi_sony_s6d05a1_hvga.o
obj-y += mddi_hitachi_r61529_hvga.o
obj-y += mddi_sii_r61529_hvga.o
obj-y += mddi_auo_s6d05a1_hvga.o
obj-y += mddi_nt_tmd_mdp42.o
obj-y += mddi_nt_sharp_ls040t8lx01.o
obj-y += mddi_nt_sharp_ls042t3lx.o
obj-y += mddi_nt_sony_acx424akm.o
obj-y += mddi_nt_sony_acx427ak.o
obj-y += mddi_nt_sony_acx424ak.o
obj-y += mddi_nt_hitachi_dx09d09vm.o
obj-y += mddi_nt_sharp_ls033t3lx01.o
obj-y += mddi_nt_tmd_lt033mdv1000.o
obj-y += mddi_nt35560_fwvga.o
obj-y += mddi_nt35582_wvga.o
else
obj-$(CONFIG_FB_MSM_MDDI_NOVATEK_FWVGA) += mddi_novatek_fwvga.o
obj-$(CONFIG_FB_MSM_MDDI_SONY_HVGA) += mddi_sony_s6d05a1_hvga.o
obj-$(CONFIG_FB_MSM_MDDI_HITACHI_HVGA) += mddi_hitachi_r61529_hvga.o
obj-$(CONFIG_FB_MSM_MDDI_SII_HVGA) += mddi_sii_r61529_hvga.o
obj-$(CONFIG_FB_MSM_MDDI_AUO_HVGA) += mddi_auo_s6d05a1_hvga.o
obj-$(CONFIG_FB_MSM_MDDI_NOVATEK_TMD_MDP42) += mddi_nt_tmd_mdp42.o
obj-$(CONFIG_FB_MSM_MDDI_NOVATEK_SHARP_LS040T8LX01) += mddi_nt_sharp_ls040t8lx01.o
obj-$(CONFIG_FB_MSM_MDDI_NOVATEK_SHARP_LS042T3LX) += mddi_nt_sharp_ls042t3lx.o
obj-$(CONFIG_FB_MSM_MDDI_NOVATEK_SONY_ACX424AKM) += mddi_nt_sony_acx424akm.o
obj-$(CONFIG_FB_MSM_MDDI_NOVATEK_SONY_ACX427AK) += mddi_nt_sony_acx427ak.o
obj-$(CONFIG_FB_MSM_MDDI_NOVATEK_SONY_ACX424AK) += mddi_nt_sony_acx424ak.o
obj-$(CONFIG_FB_MSM_MDDI_NOVATEK_HITACHI_DX09D09VM) += mddi_nt_hitachi_dx09d09vm.o
obj-$(CONFIG_FB_MSM_MDDI_NOVATEK_SHARP_LS033T3LX01) += mddi_nt_sharp_ls033t3lx01.o
obj-$(CONFIG_FB_MSM_MDDI_NOVATEK_TMD_LT033MDV1000) += mddi_nt_tmd_lt033mdv1000.o
obj-$(CONFIG_FB_MSM_MDDI_NT35560_FWVGA) += mddi_nt35560_fwvga.o
obj-$(CONFIG_FB_MSM_MDDI_NT35582_WVGA) += mddi_nt35582_wvga.o
endif

obj-$(CONFIG_FB_MSM_LCDC_PANEL) += lcdc_panel.o
obj-$(CONFIG_FB_MSM_HDMI_ADV7520_PANEL) += adv7520.o
obj-$(CONFIG_FB_MSM_HDMI_MSM_PANEL) += hdmi_msm.o
obj-$(CONFIG_FB_MSM_EXT_INTERFACE_COMMON) += external_common.o

obj-$(CONFIG_FB_MSM_TVOUT) += tvout_msm.o

obj-$(CONFIG_FB_MSM_WRITEBACK_MSM_PANEL) += mdp4_wfd_writeback_panel.o
obj-$(CONFIG_FB_MSM_WRITEBACK_MSM_PANEL) += mdp4_wfd_writeback.o
obj-$(CONFIG_FB_MSM_WRITEBACK_MSM_PANEL) += mdp4_overlay_writeback.o
endif

obj-$(CONFIG_MSM_VIDC_1080P) += vidc/
obj-$(CONFIG_MSM_VIDC_720P) += vidc/
clean:
	rm *.o .*cmd
