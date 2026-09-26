#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

pub fn start() -> Result<(), JsValue> {
    console_error_panic_hook::set_once();
    let _ = tracing_subscriber::fmt::try_init();

    wasm_bindgen_futures::spawn_local(async {
        use wasm_bindgen::JsCast;

        let web_options = eframe::WebOptions::default();
        let document = web_sys::window()
            .and_then(|win| win.document())
            .expect("Failed to get DOM document");
        let canvas = document
            .get_element_by_id("kine_canvas")
            .expect("Failed to locate canvas element #kine_canvas")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("Element #kine_canvas is not a HTMLCanvasElement");

        let start_result = eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|_cc| Ok(Box::new(crate::ui::RoboSimApp::default()))),
            )
            .await;

        if let Some(loading_screen) = document.get_element_by_id("loading_screen") {
            match start_result {
                Ok(_) => {
                    loading_screen.remove();
                }
                Err(e) => {
                    loading_screen.set_inner_html(
                        "<p style=\"color:#ef4444; font-weight:bold;\">Failed to initialize WebGL/WebAssembly canvas. Please check browser console.</p>",
                    );
                    panic!("Failed to launch eframe WebRunner: {e:?}");
                }
            }
        }
    });

    Ok(())
}
