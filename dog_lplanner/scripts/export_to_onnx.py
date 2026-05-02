"""
Export a trained SAC Actor checkpoint to ONNX for deployment.

Requires:
  PyTorch >= 1.9.0, torch.onnx (built-in), optional onnxruntime for --verify.

Examples (paths relative to training repo root):
  python scripts/export_to_onnx.py --model_path data/models/sac_actor.pth --config_path configs/a1.yaml
  python scripts/export_to_onnx.py ... --verify

Fresh environment (if needed):
  pip install torch>=1.9.0 pyyaml numpy
  pip install onnxruntime  # optional, for --verify
"""
import argparse
import torch
import yaml
import numpy as np
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from policy.SAC.SAC_actor import DiagGaussianActor


class DeterministicActorWrapper(torch.nn.Module):
    """Wrap DiagGaussianActor: export tanh(mean) only (ONNX has no distributions)."""
    def __init__(self, actor):
        super().__init__()
        self.actor = actor
        
    def forward(self, obs):
        """obs [B, obs_dim] -> action [B, 3] in [-1, 1] (no vx/vy in obs, only w among velocities)."""
        mu, log_std = self.actor.trunk(obs).chunk(2, dim=-1)
        log_std = torch.tanh(log_std)
        log_std_min, log_std_max = self.actor.log_std_bounds
        log_std = log_std_min + 0.5 * (log_std_max - log_std_min) * (log_std + 1)
        action = torch.tanh(mu)
        
        return action


def export_to_onnx(
    model_path,
    config_path,
    output_path,
    opset_version=13,
    verify=True,
    verbose=True
):
    """Load checkpoint + config, trace wrapped actor, write ONNX; optionally verify with onnxruntime."""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    sac_config = config.get('sac', {})
    actor_config = sac_config.get('actor', {})
    cmd_scale = config.get('cmd_scale', [0.8, 0.4, 0.35])

    temp_state_dict = torch.load(model_path, map_location='cpu')
    actor_obs_dim = temp_state_dict['trunk.0.weight'].shape[1]

    hidden_sizes = []
    layer_idx = 0
    while f'trunk.{layer_idx}.weight' in temp_state_dict:
        weight = temp_state_dict[f'trunk.{layer_idx}.weight']
        if len(weight.shape) == 2:
            hidden_sizes.append(weight.shape[0])
        layer_idx += 2

    if hidden_sizes and hidden_sizes[-1] == 6:
        hidden_sizes = hidden_sizes[:-1]

    if hidden_sizes:
        hidden_dim = hidden_sizes
        hidden_depth = len(hidden_sizes)
    else:
        hidden_dim = actor_config.get('hidden_dim', [512, 512, 256])
        hidden_depth = len(hidden_dim) if isinstance(hidden_dim, list) else actor_config.get('hidden_depth', 3)
    
    action_dim = 3

    log_std_bounds = actor_config.get('log_std_bounds', [-2, 2])
    log_std_min, log_std_max = log_std_bounds[0], log_std_bounds[1]
    
    if verbose:
        print("=" * 70)
        print("EXPORT SAC ACTOR -> ONNX")
        print("=" * 70)
        print("Configuration:")
        print(f"  Actor obs dim: {actor_obs_dim}")
        print(f"  Action dim: {action_dim}")
        print(f"  Hidden dim: {hidden_dim}")
        print(f"  Hidden depth: {hidden_depth}")
        print(f"  Log std bounds: [{log_std_min}, {log_std_max}]")
        print(f"  Model path: {model_path}")
        print(f"  Output path: {output_path}")
        print()
    
    device = torch.device('cpu')

    actor = DiagGaussianActor(
        obs_dim=actor_obs_dim,
        action_dim=action_dim,
        hidden_dim=hidden_dim,
        hidden_depth=hidden_depth,
        log_std_bounds=(log_std_min, log_std_max)
    )

    if verbose:
        print(f"Loading weights from {model_path}...")

    state_dict = torch.load(model_path, map_location=device)
    actor.load_state_dict(state_dict)
    actor.eval()

    wrapped_actor = DeterministicActorWrapper(actor)
    wrapped_actor.eval()

    dummy_input = torch.randn(1, actor_obs_dim)

    if verbose:
        print("Sanity check on CPU...")
        with torch.no_grad():
            test_output = wrapped_actor(dummy_input)
            print(f"  Input shape: {dummy_input.shape}")
            print(f"  Output shape: {test_output.shape}")
            print(f"  Output range: [{test_output.min().item():.3f}, {test_output.max().item():.3f}]")
            print("  (expected range ~ [-1, 1])")
        print()

    if verbose:
        print(f"Exporting ONNX (opset {opset_version})...")
    
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    torch.onnx.export(
        wrapped_actor,
        dummy_input,
        str(output_path),
        input_names=['observation'],
        output_names=['action'],
        dynamic_axes={
            'observation': {0: 'batch_size'},
            'action': {0: 'batch_size'}
        },
        opset_version=opset_version,
        do_constant_folding=True,
        export_params=True,
        verbose=verbose
    )

    if verbose:
        print(f"Wrote {output_path}")
        print(f"  File size: {output_path.stat().st_size / 1024 / 1024:.2f} MB")
        print()

    if verify:
        try:
            import onnxruntime as ort
            
            if verbose:
                print("Verifying ONNX with onnxruntime...")

            session = ort.InferenceSession(str(output_path))

            input_name = session.get_inputs()[0].name
            output_name = session.get_outputs()[0].name
            input_shape = session.get_inputs()[0].shape
            output_shape = session.get_outputs()[0].shape
            
            if verbose:
                print(f"  Input name: {input_name}")
                print(f"  Input shape: {input_shape}")
                print(f"  Output name: {output_name}")
                print(f"  Output shape: {output_shape}")
            
            test_input = np.random.randn(1, actor_obs_dim).astype(np.float32)
            outputs = session.run([output_name], {input_name: test_input})
            output = outputs[0]
            
            if verbose:
                print(f"  Test input shape: {test_input.shape}")
                print(f"  Test output shape: {output.shape}")
                print(f"  Test output range: [{output.min():.3f}, {output.max():.3f}]")
                
                if output.min() >= -1.1 and output.max() <= 1.1:
                    print("  OK: output roughly in [-1, 1]")
                else:
                    print(f"  WARNING: output outside [-1, 1]")

                if output.shape == (1, 3):
                    print("  OK: output shape [1, 3]")
                else:
                    print(f"  WARNING: unexpected output shape: {output.shape}")

            with torch.no_grad():
                torch_output = wrapped_actor(torch.from_numpy(test_input))
                torch_output_np = torch_output.numpy()
                
                max_diff = np.abs(output - torch_output_np).max()
                
                if verbose:
                    print(f"  Max abs diff vs PyTorch: {max_diff:.6f}")
                    if max_diff < 1e-5:
                        print("  OK: ONNX matches PyTorch")
                    elif max_diff < 1e-3:
                        print("  OK: ONNX close to PyTorch")
                    else:
                        print("  WARNING: large numerical diff vs PyTorch")

            if verbose:
                print("Verification finished.")

        except ImportError:
            if verbose:
                print("onnxruntime not installed; skip --verify (pip install onnxruntime)")
        except Exception as e:
            if verbose:
                print(f"Verification error: {e}")

    if verbose:
        print()
        print("=" * 70)
        print("DONE")
        print("=" * 70)
        print(f"Saved: {output_path}")
        print()
        print(f"Input size: {actor_obs_dim} features")
        if actor_obs_dim == 48:
            print("  Legacy 48-dim obs (includes vx, w, ...). Match training stack.")
        elif actor_obs_dim == 47:
            print("  Standard 47-dim obs (no vx/vy in lidar block; includes w).")
        elif actor_obs_dim > 47 and (actor_obs_dim - 47) % 3 == 0:
            hist = (actor_obs_dim - 47) // 3
            print(f"  With action history: 47 + {hist}*3 = {actor_obs_dim}")
        print()
        print("Next:")
        print("  1. Copy ONNX into the robot inference workspace (e.g. dog_lplanner).")
        print("  2. Load with onnxruntime, feed [batch, obs_dim], read [batch, 3] in [-1,1].")
        print(f"  3. Scale by cmd_scale: vx*= {cmd_scale[0]}, vy*= {cmd_scale[1]}, w*= {cmd_scale[2]}")
        print()
    
    return output_path


def main():
    parser = argparse.ArgumentParser(description='Export SAC Actor weights to ONNX')
    parser.add_argument(
        '--model_path',
        type=str,
        default='data/models/sac_actor.pth',
        help='Path to sac_actor.pth (or compatible) checkpoint',
    )
    parser.add_argument(
        '--config_path',
        type=str,
        default='configs/a1.yaml',
        help='Training YAML (e.g. configs/a1.yaml)',
    )
    parser.add_argument(
        '--output_path',
        type=str,
        default='sac_actor.onnx',
        help='Output .onnx path',
    )
    parser.add_argument(
        '--opset_version',
        type=int,
        default=13,
        help='ONNX opset (default 13)',
    )
    parser.add_argument(
        '--verify',
        action='store_true',
        help='Run onnxruntime sanity check (requires pip install onnxruntime)',
    )
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Less console output',
    )
    
    args = parser.parse_args()
    
    config_file = Path(args.config_path)
    config_candidates = [
        config_file,
        Path.cwd() / config_file,
        PROJECT_ROOT / config_file,
        PROJECT_ROOT / "configs" / config_file.name,
    ]
    config_path = None
    for p in config_candidates:
        if p.resolve().exists() and p.resolve().is_file():
            config_path = p.resolve()
            break
    if config_path is None:
        raise FileNotFoundError(
            f"Config file not found: {args.config_path}\n"
            f"Tried: {[str(p) for p in config_candidates]}"
        )
    
    model_file = Path(args.model_path)
    model_candidates = [
        model_file,
        Path.cwd() / model_file,
        PROJECT_ROOT / model_file,
        PROJECT_ROOT / "data" / "models" / model_file.name,
    ]
    model_path = None
    for p in model_candidates:
        if p.resolve().exists() and p.resolve().is_file():
            model_path = p.resolve()
            break
    if model_path is None:
        raise FileNotFoundError(
            f"Model file not found: {args.model_path}\n"
            f"Tried: {[str(p) for p in model_candidates]}"
        )
    
    output_path = args.output_path
    if not Path(output_path).is_absolute():
        output_path = str(Path.cwd() / output_path)
    
    export_to_onnx(
        model_path=str(model_path),
        config_path=str(config_path),
        output_path=output_path,
        opset_version=args.opset_version,
        verify=args.verify,
        verbose=not args.quiet
    )


if __name__ == '__main__':
    main()
